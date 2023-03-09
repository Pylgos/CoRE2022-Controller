extends Panel


@onready var pitch_label: Label = $HBoxContainer/VBoxContainer/VBoxContainer/PitchLabel
@onready var yaw_label: Label = $HBoxContainer/VBoxContainer/VBoxContainer/YawLabel
@onready var opentrack_state: Label = $HBoxContainer/VBoxContainer/OpentrackControl/OpentrackState
@onready var run_opentrack_button: Button = $HBoxContainer/VBoxContainer/OpentrackControl/RunOpentrackButton
@onready var opentrack_enable_button: CheckButton = $HBoxContainer/VBoxContainer/OpentrackControl/OpentrackEnableButton
@onready var camera_preset_manager: VBoxContainer = $HBoxContainer/CameraPresetManager
@onready var default_pitch_slider: VSlider = $HBoxContainer/DefaultPitchSlider
@onready var default_pitch_label: Label = $HBoxContainer/VBoxContainer/VBoxContainer/DefaultPitchLabel


@onready var _default_camera_pitch := deg_to_rad(default_pitch_slider.value)
var _using_default_pitch := false

func _ready() -> void:
    _update_labels()
    Robot.camera_angle_changed.connect(_update_labels)
    
    Input.joy_connection_changed.connect(_detect_opentrack)
    _detect_opentrack()
    run_opentrack_button.pressed.connect(func():
        OS.create_process("opentrack", [])
    )
    
    camera_preset_manager.preset_selected.connect(func(angle: Vector2):
        opentrack_enable_button.button_pressed = false
        if is_nan(angle.x):
            angle.x = _default_camera_pitch
            _using_default_pitch = true
        Robot.set_camera_angle(angle.x, angle.y)
    )
    
    default_pitch_slider.value_changed.connect(func(value):
        _default_camera_pitch = deg_to_rad(value)
        _update_labels()
        if _using_default_pitch:
            Robot.set_camera_angle(_default_camera_pitch, Robot.get_camera_yaw())
    )


func _update_labels() -> void:
    var pitch_deg := rad_to_deg(Robot.get_camera_pitch())
    var yaw_deg := rad_to_deg(Robot.get_camera_yaw())
    pitch_label.text = "Pitch %7.1f [deg]" % pitch_deg
    yaw_label.text = "Yaw   %7.1f [deg]" % yaw_deg
    default_pitch_label.text = "Default Pitch %7.0f [deg]" % rad_to_deg(_default_camera_pitch)

var _opentrack_joy_id := -1


func _detect_opentrack(_device := -1, _connected := false):
    _opentrack_joy_id = -1
    for id in Input.get_connected_joypads():
        if "opentrack headpose" == Input.get_joy_name(id):
            _opentrack_joy_id = id
    if _opentrack_joy_id == -1:
        opentrack_state.text = "Opentrack Not Found"
        opentrack_state.self_modulate = Color.RED
    else:
        opentrack_state.text = "Opentrack Found"
        opentrack_state.self_modulate = Color.GREEN

var camera_angle_before_lock: Vector2

func _input(_event: InputEvent) -> void:
#    var look_dir = Input.get_axis("look_right", "look_left") * PI / 2
    
    if Input.is_action_pressed("camera_lock"):
        Robot.set_camera_angle(_default_camera_pitch, 0)
        _using_default_pitch = false
    
    elif opentrack_enable_button.button_pressed and _opentrack_joy_id != -1:
        var rpy := Vector3.ZERO
        @warning_ignore("int_as_enum_without_cast")
        rpy.z = -Input.get_joy_axis(_opentrack_joy_id, 3) * PI
        @warning_ignore("int_as_enum_without_cast")
        rpy.y = -Input.get_joy_axis(_opentrack_joy_id, 4) * PI
        @warning_ignore("int_as_enum_without_cast")
        rpy.x = Input.get_joy_axis(_opentrack_joy_id, 5) * PI
        var pitch := rpy.y
        var yaw := rpy.z
        Robot.set_camera_angle(pitch, yaw)
        _using_default_pitch = false
    
#    else:
#        Robot.set_camera_angle(0.0, look_dir)
