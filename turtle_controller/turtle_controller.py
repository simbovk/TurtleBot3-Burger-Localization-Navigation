import numpy as np
import cv2
import heapq
import json
import yaml
from controller import Robot, Motor, DistanceSensor, Camera, RangeFinder

# =================== grid <-> world ===================
def world_to_grid(xw, yw, origin, resolution, h):
    if np.isnan(xw) or np.isnan(yw) or np.isinf(xw) or np.isinf(yw):
        return -1, -1
    col = int((xw - origin[0]) / resolution)
    row = h - 1 - int((yw - origin[1]) / resolution)
    return row, col

def grid_to_world(row, col, origin, resolution, h):
    xw = origin[0] + (col + 0.5) * resolution
    yw = origin[1] + ((h - 1 - row) + 0.5) * resolution
    return xw, yw

# =================== A* ===================
def astar(grid, start, goal):
    rows, cols = grid.shape
    if not (0 <= start[0] < rows and 0 <= start[1] < cols): return None
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols): return None
    if grid[start] != 0 or grid[goal] != 0: return None
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    def h(a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx,ny] == 0:
                ng = g_score[current] + 1
                nxt = (nx, ny)
                if nxt not in g_score or ng < g_score[nxt]:
                    g_score[nxt] = ng
                    heapq.heappush(open_list, (ng + h(nxt, goal), nxt))
                    came_from[nxt] = current
    return None

# =================== Particle Filter ===================
class ParticleFilter:
    def __init__(self, num_particles, occ_grid, resolution, origin):
        self.num_particles=num_particles
        self.resolution=resolution
        self.origin=origin
        self.h,self.w=occ_grid.shape
        self.grid_occ=occ_grid
        self.Q=np.array([0.02,0.02,0.01])
        self.sigma_ds=0.1
        self.particles=np.zeros((num_particles,3))
        self.weights=np.ones(num_particles)/num_particles

        # --- initialize uniformly in free space ---
        free_cells=np.argwhere(self.grid_occ==0)
        idx=np.random.choice(len(free_cells),self.num_particles,replace=True)
        for k,(row,col) in enumerate(free_cells[idx]):
            x,y=grid_to_world(row,col,self.origin,self.resolution,self.h)
            self.particles[k,0]=x
            self.particles[k,1]=y
            self.particles[k,2]=np.random.uniform(-np.pi,np.pi)

    def reset_with_qr(self, rx, ry, theta):
        self.particles[:,0]=rx+np.random.normal(0,0.05,self.num_particles)
        self.particles[:,1]=ry+np.random.normal(0,0.05,self.num_particles)
        self.particles[:,2]=theta+np.random.normal(0,0.05,self.num_particles)
        self.weights[:]=1.0/self.num_particles
        print(f"[QR RESET] Pose=({rx:.2f},{ry:.2f},{theta:.2f})")

    def predict(self,u):
        noise=np.random.randn(self.num_particles,3)*self.Q
        self.particles+=u+noise
        self.particles[:,2]=np.arctan2(np.sin(self.particles[:,2]),np.cos(self.particles[:,2]))

    def update_ds_all(self,z_dict):
        likelihood=np.ones(self.num_particles)
        for ang,z in z_dict.values():
            exp=[]
            for p in self.particles:
                exp.append(self._raycast(p[0],p[1],p[2]+ang))
            exp=np.array(exp)
            diff=z-exp
            likelihood*=np.exp(-0.5*(diff/self.sigma_ds)**2)
        self.weights*=likelihood
        if np.sum(self.weights)==0 or np.isnan(np.sum(self.weights)):
            self.weights[:]=1.0/self.num_particles
        else:
            self.weights/=self.weights.sum()
        self.resample()

    def _raycast(self,x,y,theta,max_range=3.0):
        step=self.resolution; r=0.0
        while r<max_range:
            xx=x+r*np.cos(theta); yy=y+r*np.sin(theta)
            row,col=world_to_grid(xx,yy,self.origin,self.resolution,self.h)
            if row<0 or col<0 or row>=self.h or col>=self.w:
                return max_range
            if self.grid_occ[row,col]==1: return r
            r+=step
        return max_range

    def resample(self):
        eff=1.0/np.sum(self.weights**2)
        if eff<self.num_particles/2:
            idx=np.random.choice(np.arange(self.num_particles),size=self.num_particles,p=self.weights)
            self.particles=self.particles[idx]
            self.weights[:]=1.0/self.num_particles

    def get_est(self):
        m=np.average(self.particles,weights=self.weights,axis=0)
        if np.any(np.isnan(m)):
            m=np.mean(self.particles,axis=0)
        m[2]=np.arctan2(np.sin(m[2]),np.cos(m[2]))
        return m

# =================== Init Webots ===================
TIME_STEP=64
robot=Robot()
left_motor=robot.getDevice('left wheel motor')
right_motor=robot.getDevice('right wheel motor')
for m in [left_motor,right_motor]:
    m.setPosition(float("inf")); m.setVelocity(0.0)

ds_left=robot.getDevice("ds_left"); ds_left.enable(TIME_STEP)
ds_right=robot.getDevice("ds_right"); ds_right.enable(TIME_STEP)
ds_front=robot.getDevice("ds_front"); ds_front.enable(TIME_STEP)
ds_back=robot.getDevice("ds_back"); ds_back.enable(TIME_STEP)
camera=robot.getDevice("qr_camera"); camera.enable(TIME_STEP)
range_finder=robot.getDevice("qr_rangefinder"); range_finder.enable(TIME_STEP)
l_enc=robot.getDevice('left wheel sensor'); r_enc=robot.getDevice('right wheel sensor')
l_enc.enable(TIME_STEP); r_enc.enable(TIME_STEP)

WHEEL_RADIUS=0.033; WHEEL_BASE=0.16
MAX_SENSOR_RANGE=5.0

# =================== Load Map & Doors ===================
pgm=cv2.imread("point_cloud_run.pgm",cv2.IMREAD_GRAYSCALE)
occ=(pgm<128).astype(np.uint8)
with open("point_cloud_run.yaml") as f: yml=yaml.safe_load(f)
origin=yml["origin"][:2]; resolution=yml["resolution"]; h,w=occ.shape
with open("point_cloud_run_doors.json") as f: doors=json.load(f)["doors"]
landmarks={d["id"]:grid_to_world(d["j"],d["i"],origin,resolution,h) for d in doors}

pf=ParticleFilter(1000,occ,resolution,origin)

# =================== Helpers ===================
robot.step(TIME_STEP)  # مهم: یک استپ بزن قبل از خواندن انکودر
last_l,last_r=l_enc.getValue(),r_enc.getValue()

def odom_step():
    global last_l,last_r
    l=l_enc.getValue(); r=r_enc.getValue()
    dl=(l-last_l)*WHEEL_RADIUS; dr=(r-last_r)*WHEEL_RADIUS
    last_l,last_r=l,r
    d=(dl+dr)/2; dth=(dr-dl)/WHEEL_BASE
    dx=d*np.cos(pf.get_est()[2]+dth/2); dy=d*np.sin(pf.get_est()[2]+dth/2)
    pf.predict(np.array([dx,dy,dth]))
    print(f"[ODOM] Δl={dl:.4f}, Δr={dr:.4f}, dx={dx:.3f}, dy={dy:.3f}, dθ={dth:.3f}")

def ds_update():
    zL=(ds_left.getValue()/1000.0)*MAX_SENSOR_RANGE
    zR=(ds_right.getValue()/1000.0)*MAX_SENSOR_RANGE
    zF=(ds_front.getValue()/1000.0)*MAX_SENSOR_RANGE
    zB=(ds_back.getValue()/1000.0)*MAX_SENSOR_RANGE
    pf.update_ds_all({"L":(+np.pi/2,zL),"R":(-np.pi/2,zR),"F":(0,zF),"B":(np.pi,zB)})

# =================== QR Detection (ساده مثل کد قبلی) ===================
def detect_qr(camera):
    """Detect QR codes using OpenCV and return decoded number if found."""
    width = camera.getWidth()
    height = camera.getHeight()

    raw_image = camera.getImage()
    image_data = np.frombuffer(raw_image, dtype=np.uint8).reshape((height, width, 4))

    image_gray = cv2.cvtColor(image_data, cv2.COLOR_RGBA2GRAY)
    

    detector = cv2.QRCodeDetector()
    retval, decoded_info, points, _ = detector.detectAndDecodeMulti(image_gray)

    if retval:
        for obj in decoded_info:
            if obj:
                print(f"[QR DETECT] Found QR={obj}")
                return {'number': obj}
    return None

def qr_update():
    det = detect_qr(camera)
    if not det: 
        return
    qr_id = det['number']
    if qr_id in landmarks:
        lx, ly = landmarks[qr_id]
        est = pf.get_est()
        pf.reset_with_qr(lx, ly, est[2])
        print(f"[QR RESET] Door {qr_id} Pose=({lx:.2f},{ly:.2f})")

# =================== Navigation ===================
def plan_to_goal(goal_id):
    est=pf.get_est()
    start=world_to_grid(est[0],est[1],origin,resolution,h)
    gx,gy=landmarks[goal_id]
    goal=world_to_grid(gx,gy,origin,resolution,h)
    return astar(occ,start,goal)

def move_along(path,lookahead=5,v_lin=0.25,k_ang=2.0,steps=30):
    if not path: return False
    row,col=path[min(lookahead,len(path)-1)]
    tx,ty=grid_to_world(row,col,origin,resolution,h)
    for s in range(steps):
        est=pf.get_est()
        dx,dy=tx-est[0],ty-est[1]
        th_des=np.arctan2(dy,dx)
        ang_err=np.arctan2(np.sin(th_des-est[2]),np.cos(th_des-est[2]))
        dist=np.hypot(dx,dy)
        v=v_lin*np.clip(dist/0.5,0.2,1.0); w=k_ang*ang_err
        vl=(v-w*WHEEL_BASE/2)/WHEEL_RADIUS; vr=(v+w*WHEEL_BASE/2)/WHEEL_RADIUS
        left_motor.setVelocity(np.clip(vl,-6.28,6.28))
        right_motor.setVelocity(np.clip(vr,-6.28,6.28))
        if robot.step(TIME_STEP)==-1: break
        odom_step(); ds_update(); qr_update()
    left_motor.setVelocity(0); right_motor.setVelocity(0)
    return True

# =================== Main ===================
goal_id="2"  # Target Door

while robot.step(TIME_STEP)!=-1:
    odom_step(); ds_update(); qr_update()
    gx,gy=landmarks[goal_id]; est=pf.get_est()
    dist=np.hypot(gx-est[0],gy-est[1])
    print(f"[POSE] ({est[0]:.2f},{est[1]:.2f},{est[2]:.2f}) → Goal {goal_id} Dist={dist:.2f}")
    if dist<0.4:
        print("Goal Reached ✅")
        break
    path=plan_to_goal(goal_id)
    if path: move_along(path)