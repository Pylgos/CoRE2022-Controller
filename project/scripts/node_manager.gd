extends PanelContainer

@export var state_color_active := Color.GREEN
@export var state_color_inactive := Color.ORANGE
@export var state_color_unconfigured := Color.GRAY
@export var state_color_finalized := Color.RED

var _manager := LifecycleManager.new()
@onready var _current_state := $VBoxContainer/CurrentState

@onready var _activate_button := $VBoxContainer/Buttons/Activate
@onready var _deactivate_button := $VBoxContainer/Buttons/Deactivate
@onready var _configure_button := $VBoxContainer/Buttons/Configures
@onready var _cleanup_button := $VBoxContainer/Buttons/Cleanup

var state_color_table := {
    LifecycleManager.ACTIVE: state_color_active,
    LifecycleManager.INACTIVE: state_color_inactive,
    LifecycleManager.UNCONFIGURED: state_color_unconfigured,
    LifecycleManager.FINALIZED: state_color_finalized,
}

var transition_button_table


func _ready() -> void:
    _manager.init("can_proxy")
    _manager.state_changed.connect(_update_state)
    transition_button_table = {
        _activate_button: LifecycleManager.ACTIVATE,
        _deactivate_button: LifecycleManager.DEACTIVATE,
        _configure_button: LifecycleManager.CONFIGURE,
        _cleanup_button: LifecycleManager.CLEANUP,
    }
    for btn in transition_button_table.keys():
        btn.pressed.connect(_manager.make_transition.bind(transition_button_table[btn]))


func _update_state() -> void:
    var state = _manager.get_state()
    var state_str: String
    match state:
        LifecycleManager.ACTIVE:
            state_str = "ACTIVE"
            _deactivate_button.show()
            _activate_button.hide()
            _configure_button.hide()
            _cleanup_button.hide()
        LifecycleManager.INACTIVE:
            state_str = "INACTIVE"
            _deactivate_button.hide()
            _activate_button.show()
            _configure_button.hide()
            _cleanup_button.show()
        LifecycleManager.UNCONFIGURED:
            state_str = "UNCONFIGURED"
            _deactivate_button.hide()
            _activate_button.hide()
            _configure_button.show()
            _cleanup_button.hide()
        LifecycleManager.FINALIZED:
            state_str = "FINALIZED"
            _deactivate_button.hide()
            _activate_button.hide()
            _configure_button.hide()
            _cleanup_button.hide()

    _current_state.text = state_str
    _current_state.self_modulate = state_color_table[state]

func _process(delta: float) -> void:
    pass
