from open_micro_stage_api import OpenMicroStageInterface

oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect(port='COM3', baud_rate=115200)

# home device
oms.home()

# move and wait
oms.move_to(0, 0, 0, f=10)
oms.wait_for_stop()

# print some info
oms.read_device_state_info()