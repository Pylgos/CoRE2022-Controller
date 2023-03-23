extends Panel

var _joy := -1
var _touchpad := -1

func find_controller() -> void:
    for dev in Input.get_connected_joypads():
        var joy_name := Input.get_joy_name(dev)
        if joy_name == "PS4 Controller":
            _joy = dev
            return
    _joy = -1

func _ready() -> void:
    Input.joy_connection_changed.connect(func(_dev: int, _connected: bool): find_controller())
    find_controller()
    var tim := Timer.new()
    tim.wait_time = 1.0 / 60
    tim.one_shot = false
    tim.timeout.connect(update)
    tim.autostart = true
    add_child(tim)

func update() -> void:
    if _joy == -1: return
    var axes: PackedFloat32Array
    axes.append(-Input.get_joy_axis(_joy, JOY_AXIS_LEFT_X))
    axes.append(-Input.get_joy_axis(_joy, JOY_AXIS_LEFT_Y))
    axes.append(-Input.get_joy_axis(_joy, JOY_AXIS_RIGHT_X))
    axes.append(-Input.get_joy_axis(_joy, JOY_AXIS_RIGHT_Y))
    axes.append(-Input.get_joy_axis(_joy, JOY_AXIS_TRIGGER_LEFT) * 2 + 1)
    axes.append(-Input.get_joy_axis(_joy, JOY_AXIS_TRIGGER_RIGHT) * 2 + 1)
    
    var buttons: PackedInt32Array
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_A))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_B))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_X))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_Y))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_BACK))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_GUIDE))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_START))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_LEFT_STICK))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_RIGHT_STICK))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_LEFT_SHOULDER))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_RIGHT_SHOULDER))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_DPAD_UP))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_DPAD_DOWN))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_DPAD_LEFT))
    buttons.append(Input.is_joy_button_pressed(_joy, JOY_BUTTON_DPAD_RIGHT))
    @warning_ignore("int_as_enum_without_cast")
    buttons.append(Input.is_joy_button_pressed(_touchpad, 2))
    
    Robot.send_joy_message(axes, buttons)

func _input(event: InputEvent) -> void:
    var device_name := Input.get_joy_name(event.device)
    if device_name == "PS4 Controller" and _joy != event.device:
        _touchpad = event.device
