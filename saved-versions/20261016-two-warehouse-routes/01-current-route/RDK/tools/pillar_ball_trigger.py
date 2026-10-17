"""Normal full-ball detection for the pillar, without early or position gates."""


class PillarBallTrigger:
    state = 'WAIT_VALID_BALL'

    def update(self, result):
        if result.valid and result.bbox is not None:
            self.state = 'BALL_FOUND'
            return 'TRIGGER_VALID'
        self.state = 'WAIT_VALID_BALL'
        return None
