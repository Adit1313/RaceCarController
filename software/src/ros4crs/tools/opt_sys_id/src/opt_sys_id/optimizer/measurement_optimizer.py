from .subtrajectory_optimizer import SubtrajectoryOptimizer

from opt_sys_id.model.awd_model import AwdBicycleModel


class MeasurementOptimizer(SubtrajectoryOptimizer):
    def __init__(self, model: AwdBicycleModel, dt: float, N: int) -> None:
        super().__init__(model, dt, N, N)
