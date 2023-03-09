extends Panel

@onready var lift_up_button: Button = $VBoxContainer/LiftButtons/LiftUpButton
@onready var lift_down_button: Button = $VBoxContainer/LiftButtons/LiftDownButton
@onready var grab_button: Button = $VBoxContainer/GrabButton

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    lift_down_button.button_down.connect(func():
        Robot.set_arm_lift_command(-1)
    )
    lift_down_button.button_up.connect(func():
        Robot.set_arm_lift_command(0)
    )
    lift_up_button.button_down.connect(func():
        Robot.set_arm_lift_command(1)
    )
    lift_up_button.button_up.connect(func():
        Robot.set_arm_lift_command(0)
    )
    
    grab_button.button_down.connect(func():
        Robot.set_arm_grabber_command(1)
    )
    grab_button.button_up.connect(func():
        Robot.set_arm_grabber_command(0)
    )
