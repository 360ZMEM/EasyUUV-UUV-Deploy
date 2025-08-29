serial_id = 'COM8' # An example
api_key = 'your_api_ley_here'
api_base_url = 'https://api.openai.com/v1'
enable_LLM = True
model_name = 'gpt-4o'

ep_step = 240
GOAL_TYPE = 'Triang'  # 'Triang' / 'Step'
DRAW_PIC_realtime = False # enable plt.ion() ?
action_ratio = [0.35, 0.35, 0.35] # roll, pitch, yaw

LLM_wait_time = 7.5 # query LLM 7.5s after previous adjustment