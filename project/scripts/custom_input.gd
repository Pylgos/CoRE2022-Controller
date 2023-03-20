extends Node

#var removed_device: Array[int]
#var event_buffer: Array[InputEvent]
#
#func _press_acation(positive: StringName, negative: StringName, value: float) -> void:
#    var event: InputEventAction = null
#    if value >= 0:
#        if InputMap.action_get_deadzone(positive) < value:
#            event = InputEventAction.new()
#            Input.action_press(positive, Input.get_action_raw_strength(positive) + value)
#            Input.action_press(negative, Input.get_action_raw_strength(negative))
#            event.set_action(positive)
#    else:
#        if InputMap.action_get_deadzone(negative) < -value:
#            event = InputEventAction.new()
#            Input.action_press(positive, Input.get_action_raw_strength(positive))
#            Input.action_press(negative, Input.get_action_raw_strength(negative) - value)
#            event.set_action(negative)
#    if event != null:
#        Input.parse_input_event(event)
#
#
#func _process(_delta: float) -> void:
#    for device in Input.get_connected_joypads():
#        if device in removed_device: continue
#
#        _press_acation("right", "left", Input.get_joy_axis(device, JOY_AXIS_LEFT_X))
#        _press_acation("backward", "forward", Input.get_joy_axis(device, JOY_AXIS_LEFT_Y))


#func _input(event: InputEvent) -> void:
#    if event.device in removed_device: return
#    var device_name := Input.get_joy_name(event.device)
#    print(device_name)
#    if "Touchpad" in device_name or "TouchScreen" in device_name or "opentrack headpose" in device_name:
#        print("Input device '%s' does not look like a joypad." % [device_name])
#        removed_device.push_back(event.device)
