from statemachine import StateMachine, State

class FoamCuttingMachine(StateMachine):

    # define states
    standby = State('Standby', initial=True)
    moveToStartPose = State('MoveToStartPose')
    mountWire = State('MountWire')
    tensionWire = State('TensionWire')
    cutting = State('Cutting')

    # define transitions
    start_syncronizing = standby.to(moveToStartPose)
    start_mounting = moveToStartPose.to(mountWire)
    start_tensioning = mountWire.to(tensionWire)
    start_cutting = tensionWire.to(cutting)
    end_cutting = cutting.to(standby)

    def on_enter_moveToStartPose(self):
        print('State changed to moveToStartPose')

    def on_enter_mountWire(self):
        print('State changed to mountWire')

    def on_enter_tensionWire(self):
        print('State changed to tensionWire')

    def on_enter_cutting(self):
        print('State changed to cutting')

    def on_enter_standby(self):
        print('State changed to standby')
