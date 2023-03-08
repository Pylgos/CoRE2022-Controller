extends Panel


@onready var pitch_label: Label = $HBoxContainer/VBoxContainer/VBoxContainer/PitchLabel
@onready var yaw_label: Label = $HBoxContainer/VBoxContainer/VBoxContainer/YawLabel
@onready var presets_container: HFlowContainer = $HBoxContainer/VBoxContainer2/PresetsContainer
@onready var opentrack_state: Label = $HBoxContainer/VBoxContainer/OpentrackControl/OpentrackState
@onready var run_opentrack_button: Button = $HBoxContainer/VBoxContainer/OpentrackControl/RunOpentrackButton
@onready var opentrack_enable_button: CheckButton = $HBoxContainer/VBoxContainer/OpentrackControl/OpentrackEnableButton


var presets := {
    "Forward": Vector2(0, 0),
    "Backward": Vector2(0, PI),
    "Left": Vector2(0, PI/2),
    "Right": Vector2(0, -PI/2),
}


func _ready() -> void:
    _update_labels()
    for preset_name in presets:
        _add_preset_btn(preset_name, presets[preset_name].x, presets[preset_name].y)
    Robot.camera_angle_changed.connect(_update_labels)
    
    Input.joy_connection_changed.connect(_detect_opentrack)
    _detect_opentrack()
    run_opentrack_button.pressed.connect(func():
        OS.create_process("opentrack", [])
    )


func _update_labels() -> void:
    var pitch_deg := rad_to_deg(Robot.get_camera_pitch())
    var yaw_deg := rad_to_deg(Robot.get_camera_yaw())
    pitch_label.text = "Pitch %7.1f [deg]" % pitch_deg
    yaw_label.text = "Yaw   %7.1f [deg]" % yaw_deg


func _create_preset_btn(preset_name: String, pitch: float, yaw: float) -> Button:
    var btn = Button.new()
    btn.text = preset_name
    btn.pressed.connect(func():
        Robot.set_camera_angle(pitch, yaw)
    )
    return btn


func _add_preset_btn(preset_name: String, pitch: float, yaw: float) -> void:
    var btn := _create_preset_btn(preset_name, pitch, yaw)
    presets_container.add_child(btn)


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


func _input(_event: InputEvent) -> void:
    if _opentrack_joy_id == -1: return
    if not opentrack_enable_button.button_pressed: return
    var rpy := Vector3.ZERO
    @warning_ignore("int_as_enum_without_cast")
    rpy.z = -Input.get_joy_axis(_opentrack_joy_id, 3)
    @warning_ignore("int_as_enum_without_cast")
    rpy.y = -Input.get_joy_axis(_opentrack_joy_id, 4)
    @warning_ignore("int_as_enum_without_cast")
    rpy.x = Input.get_joy_axis(_opentrack_joy_id, 5)
    var pitch := rpy.y
    var yaw := rpy.z
    Robot.set_camera_angle(pitch, yaw)
