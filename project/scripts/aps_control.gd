extends Panel

@onready var check_button: CheckButton = $CheckButton
@onready var aps_indicator: ColorRect = $"../../LauncherManager/ApsIndicator"

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    Robot.aps_state_changed.connect(func():
        check_button.button_pressed = Robot.get_aps_enabled()
        update_indicator(check_button.button_pressed)
    )
    check_button.pressed.connect(func():
        Robot.set_aps_enabled(check_button.button_pressed)
        update_indicator(check_button.button_pressed)
    )


func update_indicator(s: bool) -> void:
    if s:
        aps_indicator.color = Color.ORANGE
    else:
        aps_indicator.color = Color.GREEN
