import os
import sys
import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import torch
import serial
import time 
from q_math import euler_xyz_from_quat, quat_from_euler_xyz
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

import base64
import openai
from collections import deque
import matplotlib.patheffects as path_effects
import threading
import pickle

from prompt import prompt_text
from configs import *

# API key
os.environ["OPENAI_API_KEY"] = api_key
client = openai.OpenAI(base_url= api_base_url)


import re

from datetime import datetime

def get_latest_image_filename(directory):
    # match the newest visual log
    pattern = re.compile(r'^log_(\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2})\.png$')
    
    latest_time = None
    latest_file = None

    for filename in os.listdir(directory):
        match = pattern.match(filename)
        if match:
            time_str = match.group(1)
            try:
                current_time = datetime.strptime(time_str, "%Y_%m_%d_%H_%M_%S")
            except ValueError:
                continue

            if latest_time is None or current_time > latest_time:
                latest_time = current_time
                latest_file = filename

    return latest_file

def extract_python_code_blocks(text):
    pattern = re.compile(r'```python\n(.*?)```', re.DOTALL)
    return [match.group(1) for match in pattern.finditer(text)]
def encode_image(image_path):
    """encode image to BASE64"""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")

def openai_analyze_image(image_path, question):
    base64_image = encode_image(image_path)
    
    response = client.chat.completions.create(
        model=model_name,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": question
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{base64_image}"
                        }
                    }
                ]
            }
        ],
        max_tokens=800 
    )

    return response.choices[0].message.content


# Parse ESP32 serial output 
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

def quat_diff(quat1:np.ndarray, quat2:np.ndarray) -> np.ndarray:
    q1_conjugate = quat1.copy()
    q1_conjugate[:, 1:] *= -1  # 共轭四元数
    diff = np.zeros_like(quat1)
    for i in range(len(quat1)):
        diff[i, 0] = quat2[i, 0] * q1_conjugate[i, 0] - quat2[i, 1] * q1_conjugate[i, 1] - quat2[i, 2] * q1_conjugate[i, 2] - quat2[i, 3] * q1_conjugate[i, 3]
        diff[i, 1] = quat2[i, 0] * q1_conjugate[i, 1] + quat2[i, 1] * q1_conjugate[i, 0] + quat2[i, 2] * q1_conjugate[i, 3] - quat2[i, 3] * q1_conjugate[i, 2]
        diff[i, 2] = quat2[i, 0] * q1_conjugate[i, 2] - quat2[i, 1] * q1_conjugate[i, 3] + quat2[i, 2] * q1_conjugate[i, 0] + quat2[i, 3] * q1_conjugate[i, 1]
        diff[i, 3] = quat2[i, 0] * q1_conjugate[i, 3] + quat2[i, 1] * q1_conjugate[i, 2] - quat2[i, 2] * q1_conjugate[i, 1] + quat2[i, 3] * q1_conjugate[i, 0]
    return diff
def multi_model_LLM():
    image_path = get_latest_image_filename(os.getcwd() + '/hist_log/')
    goal = np.array(record_object.record_goal[-1]) * 180 / np.pi; real = np.array(record_object.real_pose[-1]) * 180 / np.pi
    new_question = prompt_text.replace('<mse_value', mse_hist.__str__())
    new_question = prompt_text.replace('<imu_obs1>', goal.__str__()).replace('<imu_obs2>', real.__str__())
    dof_str = ['\nRoll (P/I/D):', '\nPitch (P/I/D):', '\nYaw (P/I/D):']; pid_str = ''
    val_str = ['roll_paras_hist', 'pitch_paras_hist', 'yaw_paras_hist']
    for i in range(3):
        if control_mode[i]:
            pid_str += dof_str[i]
            pid_hist = eval(val_str[i]); vlen = len(pid_hist); vbeg = max(0, vlen - 2)
            for idx, j in enumerate(range(vbeg, vlen)):
                pid_str += f'Stage {idx+1} - {pid_hist[j].__str__()} / '
    new_question = new_question.replace('<roll_control>', 'Enable' if control_mode[0] else 'Disable').replace('<pitch_control>', 'Enable' if control_mode[1] else 'Disable').replace('<yaw_control>', 'Enable' if control_mode[2] else 'Disable').replace('<final_pid_value>', pid_str)
    answer = openai_analyze_image(os.getcwd() + '/hist_log/' + image_path, new_question)
    time_str = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    with open(f'hist_log/LLM_output_{time_str}.md', 'a', encoding='utf-8') as f:
        f.write(answer)
    para_block = extract_python_code_blocks(answer)[0]
    print('\n-------------\nLLM output OK!\n-------------\n')
    try:
        out = eval(para_block)
        if type(out) != dict:
            raise ValueError
        LLM_output[0] = out
        LLM_answer[0] = True
        LLM_change_time.append(t[-1])
    except:
        env.halt() # process termination
        raise ValueError 
    

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
    def step(self, action):
        roll, pitch, yaw = euler_xyz_from_quat(self.pose_q.unsqueeze(0))
        action_lim = torch.Tensor(action_ratio).to(device) 
        action = action[:3] * action_lim
        roll = angle_remap(roll + action[0]); pitch = angle_remap(pitch + action[1]); yaw = angle_remap(yaw + action[2])
        roll = roll.item() / np.pi * 180; pitch = pitch.item() / np.pi * 180; yaw = yaw.item() / np.pi * 180
        self.ser.write(f'r{round(roll, 2)}p{round(pitch, 2)}y{round(yaw, 2)}'.encode('utf-8'))
        return self.get_obs()
    
    def halt(self):
        self.ser.write('e'.encode('utf-8'))

if __name__ == "__main__":


    env = Env()

    policy = torch.jit.load('policy.pt')

    goal_list = [
        ([0, 0, 0], [0, 0, 0]),
        ([0, 0, np.pi/2.5], [0, 0, 0]),
        ([0, 0, np.pi/1.25], [0, 0, 0]),
        ([0, 0, np.pi/2.5], [0, 0, 0]),
        ([0, 0, 0], [0, 0, 0]),
        ([0, 0, -np.pi/2.5], [0, 0, 0]),
        ([0, 0, -np.pi/1.25], [0, 0, 0]),
        ([0, 0, -np.pi/2.5], [0, 0, 0]),
        ([0, 0, 0], [0, 0, 0]),
    ]

    roll_paras_hist = []; pitch_paras_hist = []; yaw_paras_hist = []

    mse_hist = []

    record_object = Record()

    LLM_thread = threading.Thread(target=multi_model_LLM)


    LLM_answer = [False]
    LLM_output = [None]
    LLM_change_time = [0]
    control_mode = [False, False, True]

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

        with torch.inference_mode():
            des_ang_rpy = goal_orientation
            des_ang_quat = quat_from_euler_xyz(torch.Tensor([des_ang_rpy[0]]), torch.Tensor([des_ang_rpy[1]]), torch.Tensor([des_ang_rpy[2]]))[0].to(device)
            env._goal_quat = des_ang_quat.to(device)
            env._goal_dep = torch.Tensor(goal_pos).to(device)[2].unsqueeze(0)

            action = policy(obs) 
            obs = env.step(action) 

        
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

        action = action.detach().cpu().numpy()
        action_str = f'{round(action[0],2)}'[:5] + ',' + f'{round(action[1],2)}'[:5] + ',' + f'{round(action[2],2)}'[:5]


        print(f'Time Elapse {round(time.time() - ep_start_time, 2)}s | Timestep {counter} | Action {action_ix}' + \
              f' | Targ_ang {target_ang_str} |  Real_ang {real_ang_str} | Itarg_ang {itarg_ang_str} | Iout_ang {iout_ang_str} | Action {action_str}     ') # , end='\r'
        

        t = np.array(record_object.timestamp); goal_p = np.array(record_object.record_goal);
        real_p = np.array(record_object.real_pose) 
        # keep_time = 30
        fLLM_c = LLM_change_time[-3:]; beg_idx = fLLM_c[0] 
            
        ylim_l = [-30,-30,-180]; ylim_m = [30,30,180]
        for i in range(3):
            line_dash[i].set_data(t, goal_p[:,i] * 180 / np.pi); line_real[i].set_data(t, real_p[:,i] * 180 / np.pi)
            axes[i].set_xlim(t[beg_idx], max(10, np.max(t)) + 0.5); axes[i].set_ylim([ylim_l[i], ylim_m[i]])

        
        if DRAW_PIC_realtime:
            fig.canvas.draw()           
            fig.canvas.flush_events()    

        if enable_LLM and LLM_answer[0]:
            ret_dict = LLM_output[0]
            LLM_answer[0] = False
            try:
                if control_mode[0]:
                    roll_paras = np.array([ret_dict['roll_zeta1'], ret_dict['roll_zeta2']])
                    roll_paras_hist.append(roll_paras)
                    env.ser.write(f'n{round(roll_paras[0], 2)}m{round(roll_paras[1], 2)}'.encode('utf-8'))
                if control_mode[1]:
                    pitch_paras = np.array([ret_dict['pitch_zeta1'], ret_dict['pitch_zeta2']]) 
                    pitch_paras_hist.append(pitch_paras)
                    env.ser.write(f'v{round(pitch_paras[0], 2)}b{round(pitch_paras[1], 2)}'.encode('utf-8'))
                if control_mode[2]:
                    yaw_paras = np.array([ret_dict['yaw_zeta1'], ret_dict['yaw_zeta2']])
                    yaw_paras_hist.append(yaw_paras)
                    env.ser.write(f'z{round(yaw_paras[0], 2)}x{round(yaw_paras[1], 2)}'.encode('utf-8'))
            except:
                raise ValueError
            
            LLM_thread = threading.Thread(target=multi_model_LLM) # reset thread
            
        if (t[-1] - LLM_change_time[-1] > LLM_wait_time) and (not LLM_thread.is_alive()):
            mse_hist.append(round(np.mean((2 * np.arccos(quat_diff(goal_p[t > LLM_change_time[-1]], real_p[t > LLM_change_time[-1]]))) ** 2), 4))
            time_str = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
            if enable_LLM:
                stage_t = []
                fLLM_c.append(t[-1])
                for i in range(len(fLLM_c) - 1):
                    stage_t.append((fLLM_c[i] + fLLM_c[i+1]) / 2.6)
                draw_object = []
                idx_offset = max((len(stage_t) - 3), 0)
                for i in range(3):
                    for idx, it in enumerate(stage_t):
                        at = axes[i].text(it, -np.pi / 2, f'Stage{idx + 1}', color='w', path_effects = [path_effects.Stroke(linewidth=1, foreground='black'), path_effects.Normal()])
                        draw_object.append(at)
                    for im in fLLM_c:
                        obj, = axes[i].plot([im,im], [-200, 200], linewidth=1.9, linestyle='--', color='#bfbfbf')
                        draw_object.append(obj)
            fig.savefig(f'hist_log/log_{time_str}.png', dpi=100)
            pickle.dump(record_object, open(f'hist_log/log_{time_str}.pkl', 'wb'))
            if enable_LLM:
                for d in draw_object:
                    d.remove() 
                draw_object = []
                LLM_thread.start()
