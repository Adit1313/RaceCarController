from casadi import vertcat, cos, sin
from acados_template import AcadosModel


def wheel_encoder_model(pacejka_model: AcadosModel):
    vx = pacejka_model.x[3]
    vy = pacejka_model.x[4]
    dyaw = pacejka_model.x[5]

    lf = pacejka_model.p[1]
    car_width = pacejka_model.p[15]
    wheel_radius = pacejka_model.p[16]

    steer = pacejka_model.p[18]

    vx_fl = vx - 0.5 * car_width * dyaw
    vy_fl = vy + lf * dyaw
    vx_wheel_fl = vx_fl * cos(steer) + vy_fl * sin(steer)

    vx_fr = vx + 0.5 * car_width * dyaw
    vy_fr = vy + lf * dyaw
    vx_wheel_fr = vx_fr * cos(steer) + vy_fr * sin(steer)

    vx_wheel_rl = vx - 0.5 * car_width * dyaw
    vx_wheel_rr = vx + 0.5 * car_width * dyaw

    # Unit of wheel speed is rad/s
    # front left, front right, rear left, rear right
    return vertcat(
        vx_wheel_fl / wheel_radius,
        vx_wheel_fr / wheel_radius,
        vx_wheel_rl / wheel_radius,
        vx_wheel_rr / wheel_radius,
    )
