from casadi import vertcat
from acados_template import AcadosModel


def imu_model(pacejka_model: AcadosModel):
    # f_expl_expr = f_dot = [v_x, v_y, dyaw, a_x, a_y, dyaw_dot]
    return vertcat(
        pacejka_model.f_expl_expr[3],
        pacejka_model.f_expl_expr[4],
        pacejka_model.f_expl_expr[2],
    )
