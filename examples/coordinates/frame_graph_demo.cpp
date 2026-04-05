#include <vulcan/vulcan.hpp>

#include <iomanip>
#include <iostream>
#include <memory>

using namespace vulcan;

int main() {
    std::cout << std::fixed << std::setprecision(6);

    FrameContext<double> ctx;

    const double t = 3600.0;
    const auto rotation = ConstantOmegaRotation::from_wgs84();
    ctx.set_ecef(rotation, t);

    const double lon = -77.0367 * constants::angle::deg2rad;
    const double lat = 38.8951 * constants::angle::deg2rad;
    ctx.set_ned(Quantity<units::rad>(lon), Quantity<units::rad>(lat));
    ctx.set_body_euler(45.0 * constants::angle::deg2rad,
                       5.0 * constants::angle::deg2rad,
                       10.0 * constants::angle::deg2rad);

    Vec3<double> v_body;
    v_body << 100.0, 0.0, 0.0;

    const auto body_to_ned = ctx.chain(FRAME_BODY, FRAME_NED);
    const auto body_to_eci = ctx.chain(FRAME_BODY, FRAME_ECI);
    const auto v_ned = ctx.transform(v_body, FRAME_BODY, FRAME_NED);
    const auto v_eci = ctx.transform(v_body, FRAME_BODY, FRAME_ECI);

    std::cout << "Body->NED chain length (direct parent-child): "
              << body_to_ned.length() << "\n";
    std::cout << "Body->ECI chain length (via ECI-root tree): "
              << body_to_eci.length() << "\n";
    std::cout << "Body->ECI path: ";
    for (size_t i = 0; i < body_to_eci.path().frames.size(); ++i) {
        const auto id = body_to_eci.path().frames[i];
        std::cout << ctx.registry().get_node(id).name;
        if (i + 1 < body_to_eci.path().frames.size()) {
            std::cout << " -> ";
        }
    }
    std::cout << "\n";

    std::cout << "Body -> NED: [" << v_ned(0) << ", " << v_ned(1) << ", "
              << v_ned(2) << "]\n";
    std::cout << "Body -> ECI: [" << v_eci(0) << ", " << v_eci(1) << ", "
              << v_eci(2) << "]\n";

    const auto sensor_q = janus::Quaternion<double>::from_euler(
        0.0, 10.0 * constants::angle::deg2rad, 0.0);
    const auto sensor_id =
        ctx.add_frame("Sensor", FRAME_BODY,
                      std::make_shared<QuaternionProvider<double>>(sensor_q));

    Vec3<double> v_sensor;
    v_sensor << 1.0, 0.0, 0.0;
    const auto v_sensor_ned = ctx.transform(v_sensor, sensor_id, FRAME_NED);

    std::cout << "Sensor -> NED: [" << v_sensor_ned(0) << ", "
              << v_sensor_ned(1) << ", " << v_sensor_ned(2) << "]\n";

    // Symbolic graph usage (Janus archetype).
    FrameContext<SymbolicScalar> sym_ctx;
    SymbolicScalar sym_gmst = sym("gmst");
    SymbolicScalar sym_lon = sym("lon");
    SymbolicScalar sym_lat = sym("lat");
    SymbolicScalar sym_yaw = sym("yaw");
    SymbolicScalar sym_pitch = sym("pitch");
    SymbolicScalar sym_roll = sym("roll");
    SymbolicScalar sym_vx = sym("vx");
    SymbolicScalar sym_vy = sym("vy");
    SymbolicScalar sym_vz = sym("vz");

    sym_ctx.set_ecef(sym_gmst);
    sym_ctx.set_ned(Quantity<units::rad, SymbolicScalar>(sym_lon),
                    Quantity<units::rad, SymbolicScalar>(sym_lat));
    sym_ctx.set_body_euler(sym_yaw, sym_pitch, sym_roll);

    Vec3<SymbolicScalar> sym_v_body;
    sym_v_body << sym_vx, sym_vy, sym_vz;

    const auto sym_v_eci = sym_ctx.transform(sym_v_body, FRAME_BODY, FRAME_ECI);

    janus::Function f_sym_body_to_eci(
        "sym_body_to_eci",
        {sym_gmst, sym_lon, sym_lat, sym_yaw, sym_pitch, sym_roll, sym_vx,
         sym_vy, sym_vz},
        {sym_v_eci(0), sym_v_eci(1), sym_v_eci(2)});

    auto sym_eval = f_sym_body_to_eci(
        {rotation.gmst(t), lon, lat, 45.0 * constants::angle::deg2rad,
         5.0 * constants::angle::deg2rad, 10.0 * constants::angle::deg2rad,
         100.0, 0.0, 0.0});

    std::cout << "Symbolic Body -> ECI eval: [" << sym_eval[0](0, 0) << ", "
              << sym_eval[1](0, 0) << ", " << sym_eval[2](0, 0) << "]\n";

    return 0;
}
