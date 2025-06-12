import mujoco
import numpy as np
from dearpygui.core import *
from dearpygui.simple import *

# Load MuJoCo model and data
model = mujoco.MjModel.from_xml_path("model/mjmodel.xml")
data = mujoco.MjData(model)

joint_names = [model.joint(i).name for i in range(model.njnt)]

def update_joint(sender, app_data, user_data):
    idx = user_data
    data.qpos[idx] = get_value(sender)
    mujoco.mj_forward(model, data)
    set_value(f"input_{idx}", data.qpos[idx])

def edit_joint(sender, app_data, user_data):
    idx = user_data
    try:
        val = float(get_value(sender))
        data.qpos[idx] = val
        mujoco.mj_forward(model, data)
        set_value(f"slider_{idx}", val)
        hide_item(sender)
    except ValueError:
        pass

def slider_double_click(sender, app_data, user_data):
    idx = user_data
    show_item(f"input_{idx}")

with window("Joint Control"):
    for i, name in enumerate(joint_names):
        add_slider_float(f"{name}", default_value=data.qpos[i], min_value=-3, max_value=3,
                         callback=update_joint, callback_data=i, tag=f"slider_{i}")
        add_input_float(f"Edit {name}", default_value=data.qpos[i], callback=edit_joint, callback_data=i, show=False, tag=f"input_{i}")

for i in range(len(joint_names)):
    set_item_double_clicked_callback(f"slider_{i}", slider_double_click, callback_data=i)

start_dearpygui() 