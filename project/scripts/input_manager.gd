extends PanelContainer


@onready var activate_button: CheckButton = $HBoxContainer/VBoxContainer/ActivateButton
@onready var max_linear_speed_label: Label = $HBoxContainer/VBoxContainer/HBoxContainer/MaxLinearSpeedLabel
@onready var max_linear_speed_slider: HSlider = $HBoxContainer/VBoxContainer/MaxLinearSpeedSlider
@onready var max_angluar_speed_label: Label = $HBoxContainer/VBoxContainer/HBoxContainer2/MaxAngluarSpeedLabel
@onready var max_angular_speed_slider: HSlider = $HBoxContainer/VBoxContainer/MaxAngularSpeedSlider


var _max_linear_speed: float
var _max_angular_speed: float


func _ready() -> void:
    max_linear_speed_slider.value_changed.connect(_update_max_linear_speed)
    max_angular_speed_slider.value_changed.connect(_update_max_angular_speed)
    activate_button.pressed.connect(_on_activate_button_pressed)
    _update_max_linear_speed(max_linear_speed_slider.value)
    _update_max_angular_speed(max_angular_speed_slider.value)


func _on_activate_button_pressed() -> void:
    Robot.set_control(activate_button.button_pressed)


func _update_max_linear_speed(value: float) -> void:
    max_linear_speed_label.text = "%2.1f m/s" % value
    _max_linear_speed = value


func _update_max_angular_speed(value: float) -> void:
    max_angluar_speed_label.text = "%2.1f rad/s" % value
    _max_angular_speed = value


func _input(_event: InputEvent) -> void:
    pass


func _process(_delta: float) -> void:
    var vx = Input.get_axis("backward", "forward") * _max_linear_speed
    var vy = Input.get_axis("right", "left") * _max_linear_speed
    var vang = Input.get_axis("turn_right", "turn_left") * _max_angular_speed
#    print(Vector2(vx, vy), " ", vang)
    Robot.set_target_velocity(Vector2(vx, vy), vang)
