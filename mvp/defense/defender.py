from scripts.Car import Car
class Defender(Car):
    def __init__(self):
        super().__init__()

    def run(self, multi_frame_case, defend_opts):
        raise NotImplementedError