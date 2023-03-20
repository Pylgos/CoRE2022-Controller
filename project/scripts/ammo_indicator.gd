extends Panel

@export var max_ammo := 40:
    set(value):
        max_ammo = value
        update_max_ammo_label()

@onready var _max_ammo_label := $HBoxContainer/MaxAmmo
@onready var _ammo_label := $HBoxContainer/Ammo
@onready var fire_command_indicator: ColorRect = $FireCommandIndicator


func _ready() -> void:
    Robot.ammo_changed.connect(update_label)
    _max_ammo_label.text = "%02d" % max_ammo
    update_label()
    update_max_ammo_label()
    Robot.fire_command_changed.connect(_update_fire_command_indicator)
    _update_fire_command_indicator()

func update_max_ammo_label() -> void:
    if _max_ammo_label == null: return
    _max_ammo_label.text = "%02d" % max_ammo

func update_label() -> void:
    _ammo_label.text = "%02d" % Robot.get_ammo()

func _update_fire_command_indicator() -> void:
    if Robot.get_fire_command():
        fire_command_indicator.color = Color.ORANGE
    else:
        fire_command_indicator.color = Color.GREEN

func _process(_delta) -> void:
    if Input.is_action_just_pressed("fire"):
        Robot.set_fire_command(true)
    
    if Input.is_action_just_released("fire"):
        Robot.set_fire_command(false)
