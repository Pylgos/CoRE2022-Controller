extends Panel

@onready var check_button: CheckButton = $CheckButton

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    Robot.aps_state_changed.connect(func():
        check_button.button_pressed = Robot.get_aps_enabled()
    )
    check_button.pressed.connect(func():
        Robot.set_aps_enabled(check_button.button_pressed)
    )
