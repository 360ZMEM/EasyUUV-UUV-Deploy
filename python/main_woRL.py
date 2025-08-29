import os
import sys
import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import torch
import serial
import time 
from q_math import euler_xyz_from_quat, quat_from_euler_xyz
device = "cpu"
from collections import deque
import pickle
from configs import *
def parse_part(part_str):
    elements = part_str.strip().split()
    result = {'P': None, 'R': None, 'Y': None}
    
    for elem in elements:
        if elem.startswith('P:'):
            try:
                result['P'] = float(elem.split(':', 1)[1])
            except (IndexError, ValueError):
                result['P'] = None
        elif elem.startswith('R:'):
            try:
                result['R'] = float(elem.split(':', 1)[1])
            except (IndexError, ValueError):
                result['R'] = None
        elif elem.startswith('Y:'):
            try:
                result['Y'] = float(elem.split(':', 1)[1])
            except (IndexError, ValueError):
                result['Y'] = None
    return result


def parse_all_parts(data_str):
    parts = data_str.strip().split('|')[:3]
    result =  [parse_part(part) for part in parts]
    if len(result) != 3:
        return [
    {'P': None, 'R': None, 'Y': None},
    {'P': None, 'R': None, 'Y': None},
    {'P': None, 'R': None, 'Y': None}
    ]
    return result
def angle_remap(angle) -> torch.Tensor:
    return (angle + torch.pi) % (2 * torch.pi) - torch.pi 

def angle_remap_np(angle) -> np.ndarray:
    return (angle + np.pi) % (2 * np.pi) - np.pi 


class Record:
    def __init__(self, max_len = 20000):
        self.timestamp = deque(maxlen=max_len)
        self.record_goal = deque(maxlen=max_len)
        self.real_pose = deque(maxlen=max_len)

class Env:
    def __init__(self):
        self._goal_quat = torch.zeros(4).to(device)
        self._goal_dep = torch.zeros(1).to(device)
        self.pose_q = torch.zeros(4).to(device)
        self.vel = torch.zeros(3).to(device)
        self.state_dict = [
            {'P': 0, 'R': 0, 'Y': 0},
            {'P': 0, 'R': 0, 'Y': 0},
            {'P': 0, 'R': 0, 'Y': 0}
        ]
        self.ser = serial.Serial(serial_id, 115200, timeout=0.05, write_timeout=0.05)
        self.ser.reset_input_buffer()
        print(f'Enabling serial {serial_id} ...')
        while True:
            if self.ser.isOpen():
                print(f'Serial {serial_id} enabled!')
                break

    def get_obs(self):
        self.position_update()
        return torch.concatenate([self._goal_quat, self._goal_dep, self.pose_q]) # 9-dim
        
    def reset(self):
        self.ser.write('a'.encode('utf-8'))
        self.ser.write('L'.encode('utf-8'))
        time.sleep(1)
        while True:
            if self.position_update():
                break
        return self.get_obs()

    def position_update(self):
        raw_line = self.ser.readline()
        env.raw_line = raw_line

        if not raw_line:
            return False
        try:
            line = raw_line.decode('utf-8').strip()
            state_dict = parse_all_parts(line)
            vals = state_dict[0]
            if type(vals['R']) == type(None):
                raise ValueError
            self.state_dict = state_dict
            self.pose_q = quat_from_euler_xyz(torch.Tensor([vals['R'] / 180 * np.pi]), torch.Tensor([vals['P'] / 180 * np.pi]), torch.Tensor([vals['Y'] / 180 * np.pi]))[0].to(device)
            return True
        except:
            return False
    def step(self):
        # roll, pitch, yaw = euler_xyz_from_quat(self.pose_q.unsqueeze(0))
        # action_lim = torch.Tensor([0,0,-0.34]).to(device) 
        # action = action[:3] * action_lim
        # roll = angle_remap(roll + action[0]); pitch = angle_remap(pitch + action[1]); yaw = angle_remap(yaw + action[2])
        # roll = roll.item() / np.pi * 180; pitch = pitch.item() / np.pi * 180; yaw = yaw.item() / np.pi * 180
        # self.ser.write(f'r{round(roll, 2)}p{round(pitch, 2)}y{round(yaw, 2)}'.encode('utf-8'))
        return self.get_obs()
    
    def halt(self):
        self.ser.write('e'.encode('utf-8'))

if __name__ == "__main__":

    env = Env()
    goal_list = [
        ([0, 0, 0], [0, 0, 0]),
        ([0, 0, np.pi/2.5], [0, 0, 0]),
        ([0, 0, np.pi/1.25], [0, 0, 0]),
        ([0, 0, np.pi/2.5], [0, 0, 0]),
        ([0, 0, 0], [0, 0, 0]),
        ([0, 0, -np.pi/2.5], [0, 0, 0]),
        ([0, 0, -np.pi/1.25], [0, 0, 0]),
        ([0, 0, -np.pi/2.5], [0, 0, 0]),
    ]

    record_object = Record()

    if DRAW_PIC_realtime:
        plt.ion()
    fig = plt.figure(figsize=(11,3.5))
    axes = [fig.add_subplot(1, 3, i+1) for i in range(3)]
    line_dash = []; line_real = []
    angle_label = ['roll', 'pitch', 'yaw']
    for i in range(3):
        lw = 1.6
        line_dash.append(axes[i].plot([],[],'b--', linewidth=lw, label=f'Target {angle_label[i]}')[0])
        line_real.append(axes[i].plot([],[],'r-', linewidth=lw, label=f'Real {angle_label[i]}')[0])
        axes[i].legend(loc='upper right')
        axes[i].set_xlabel('Time (s)'); axes[i].set_ylabel(f'{angle_label[i].capitalize()} (deg)')
        axes[i].grid(True)
    fig.tight_layout()

    ep_step = 160
    action_ix = 0; counter = 0
    obs = env.reset()
    ep_start_time = time.time()

    while action_ix < len(goal_list):
        start_time = time.time()
        counter += 1
        goal_orientation, goal_pos = goal_list[action_ix]

        if GOAL_TYPE == 'Triang':
            target_yaw = lambda t: (np.pi / 3) * np.sin(t * 6.28 / 10)
            target_roll = lambda t: 0; target_pitch = lambda t: 0
            time_elapse = time.time() - ep_start_time
            goal_orientation = [target_roll(time_elapse), target_pitch(time_elapse), target_yaw(time_elapse)]

        # Directly send target value, rather than (real + RL output) value
        r, p, y = goal_orientation; r *= (180 / np.pi); p *= (180 / np.pi); y *= (180 / np.pi)
        env.ser.write(f'r{round(r,2)}p{round(p,2)}y{round(y,2)}'.encode('utf-8'))
        env.step()

        action_ix = counter // ep_step
        goal_orientation_ = angle_remap_np(np.array(goal_orientation))
        target_ang = np.array(goal_orientation_) * 180 / np.pi
        target_ang_str = f'{round(target_ang[0],1)},{round(target_ang[1],1)},{round(target_ang[2],1)}'
        real_roll, real_pitch, real_yaw = euler_xyz_from_quat(env.pose_q.unsqueeze(0))
        real_roll = angle_remap(real_roll); real_pitch = angle_remap(real_pitch); real_yaw = angle_remap(real_yaw)
        real_ang_str = f'{round(real_roll.item() / np.pi * 180, 1)},{round(real_pitch.item() / np.pi * 180, 1)},{round(real_yaw.item() / np.pi * 180, 1)}'

        record_object.timestamp.append(time.time() - ep_start_time)
        record_object.record_goal.append(list(goal_orientation_))
        record_object.real_pose.append([real_roll.item(), real_pitch.item(), real_yaw.item()])

        itarg_ang_str = f"{round(env.state_dict[1]['R'], 1)},{round(env.state_dict[1]['P'], 1)},{round(env.state_dict[1]['Y'], 1)}"
        iout_ang_str = f"{round(env.state_dict[2]['R'], 1)},{round(env.state_dict[2]['P'], 1)},{round(env.state_dict[2]['Y'], 1)}"

        action = [0.,0.,0.]
        action_str = f'{round(action[0],2)}'[:5] + ',' + f'{round(action[1],2)}'[:5] + ',' + f'{round(action[2],2)}'[:5]


        print(f'Time Elapse {round(time.time() - ep_start_time, 2)}s | Timestep {counter} | Action {action_ix}' + \
              f' | Targ_ang {target_ang_str} |  Real_ang {real_ang_str} | Itarg_ang {itarg_ang_str} | Iout_ang {iout_ang_str} | Action {action_str}     ') # , end='\r'
        
        t = np.array(record_object.timestamp); goal_p = np.array(record_object.record_goal);
        real_p = np.array(record_object.real_pose) 

        ylim_l = [-30,-30,-180]; ylim_m = [10,10,180]
    
        if DRAW_PIC_realtime:
            fig.canvas.draw()        
            fig.canvas.flush_events()  
        for i in range(3):
            line_dash[i].set_data(t, goal_p[:,i] * 180 / np.pi); line_real[i].set_data(t, real_p[:,i] * 180 / np.pi)
            axes[i].set_xlim(t[0], max(10, np.max(t)) + 0.5); axes[i].set_ylim([ylim_l[i], ylim_m[i]])

    time_str = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    fig.savefig(f'hist_log/log_{time_str}.png', dpi=100)
    pickle.dump(record_object, open(f'hist_log/log_{time_str}.pkl', 'wb'))
