extends VBoxContainer


signal preset_selected(angle: Vector2)


@onready var presets := {
    $HBoxContainer/LeftButton: Vector2(NAN, 90),
    $HBoxContainer/RightButton: Vector2(NAN, -90),
    $HBoxContainer/VBoxContainer/Front: Vector2(NAN, 0),
    $HBoxContainer/VBoxContainer/Rear: Vector2(NAN, 180),
}


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    for btn in presets:
        btn.pressed.connect(func():
#            Robot.set_camera_angle(deg_to_rad(presets[btn].x), deg_to_rad(presets[btn].y))
            preset_selected.emit(Vector2(deg_to_rad(presets[btn].x), deg_to_rad(presets[btn].y)))
        )
