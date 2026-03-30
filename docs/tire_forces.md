# Tire force estimation and state observation for 4WID Formula Student EVs

**Every competitive torque vectoring system on a four-wheel independent drive (4WID) electric vehicle depends on real-time estimates of forces the driver never feels and sensors never directly measure.** Vertical load, lateral force, friction coefficient, slip angle, and slip ratio must all be reconstructed from indirect measurements — accelerometers, gyroscopes, wheel encoders, and the uniquely precise motor torque feedback that only electric drivetrains provide. This report synthesizes the current state of model-based estimation techniques for these parameters, with particular focus on Formula Student Electric (FSE) implementations and how each estimated quantity feeds into the three-layer torque vectoring architecture: reference generator, high-level controller, and control allocator.

The core advantage of 4WID EVs is that **motor torque is known to within 2–5% from inverter current feedback**, enabling direct computation of longitudinal tire forces from wheel dynamics — something impossible on ICE vehicles. This single fact restructures the entire estimation problem, making cascaded observer architectures (wheel-level force observers feeding vehicle-level state estimators) both practical and accurate. The dominant approach across published FSE implementations uses an EKF or UKF with bicycle/two-track models, supplemented by quasi-static load transfer models for vertical load and disturbance observers for per-wheel longitudinal forces.

---

## Vertical load estimation relies on load transfer physics and IMU data

Vertical load (Fz) at each wheel is the foundational parameter for torque vectoring — it defines the friction circle radius (μ·Fz) that constrains every control allocation decision. The dominant method in Formula Student is the **quasi-static load transfer model**, which computes Fz from measured longitudinal and lateral accelerations.

The static wheel loads follow from weight distribution: Fz_front_static = m·g·(l_r/L) per axle, split equally left-right. Longitudinal load transfer adds **ΔFz_long = m·a_x·h_CG / L**, distributed equally between left and right wheels on each axle. Lateral load transfer is more nuanced: the total lateral transfer ΔFz_lat_total = m·a_y·h_CG / t is fixed by physics, but its distribution between front and rear axles depends on three components at each axle — geometric (via roll center height), elastic (via roll stiffness), and unsprung mass contributions. The elastic component, which dominates, distributes according to front/rear roll stiffness ratio: ΔFz_elastic_f = K_φf·φ/t_f, where φ is the body roll angle computed from the roll moment equilibrium. The lateral load transfer distribution (LLTD) — typically 50–55% rear for FS cars — is a key vehicle design parameter that directly enters this calculation.

Combined loading produces a diagonal pattern: during simultaneous acceleration and cornering, load concentrates on the outer-rear wheel, creating four distinct Fz values that must be tracked in real time. AMZ Racing (ETH Zurich) confirmed they use this acceleration-based approach for their torque vectoring and traction control, computing forces at every tire from IMU signals.

More sophisticated approaches exist. **Dynamic load transfer models** incorporate roll and pitch dynamics (I_xx·φ̈ = m·a_y·h_s − K_φ·φ − C_φ·φ̇), capturing the transient lag between acceleration onset and actual load transfer completion — critical during rapid transitions in racing. **Suspension deflection-based methods** use linear potentiometers parallel to dampers, computing Fz = k_s·Δx + c_s·Δẋ from known spring-damper characteristics. The SAE 2024-01-2285 paper validated an **EKF-based Fz observer** using suspension geometry modeled as an RSSR spatial parallel mechanism on a Formula Student car, demonstrating improved accuracy over pure acceleration methods during transient maneuvers.

Filippi and Assadian (2021) formulated Fz estimation as an input estimation problem rather than state estimation, proposing two observers — a Youla Controller Output Observer and an Unbiased Minimum Variance Filter — both using accelerometer and suspension deflection sensors. Both outperformed algebraic load transfer expressions during transients. For validation, teams like Cornell Racing and E-Team Squadra Corse (Bologna) have used strain gauges on suspension arms, achieving 84–90% correlation with physical tire models, while Kistler wheel force transducers serve as ground truth in research settings.

The vehicle parameters required for load transfer computation include: total mass (170–300 kg for FS), CG height (250–350 mm), wheelbase (1500–1600 mm), track widths (1100–1300 mm), front/rear roll stiffness (500–2000 Nm/rad each, computed from spring rates, anti-roll bars, and motion ratios), roll center heights (20–80 mm), and sprung/unsprung mass split. For cars with aerodynamic packages, speed-dependent downforce terms (Fz_aero = ½·ρ·v²·C_L·A) must be added — modern FS cars generate up to 2–3× vehicle weight in downforce at speed.

---

## Known motor torque transforms longitudinal force estimation

The wheel rotational dynamics equation **J_w·dω/dt = T_motor − T_brake − F_x·R_eff** is the foundation for Fx estimation in 4WID EVs. Since T_motor is precisely known and ω is measured from motor encoders, Fx can in principle be computed directly. In practice, differentiating ω amplifies sensor noise, tire effective radius varies 1–3% with load and speed, and drivetrain losses introduce unmodeled disturbance torques.

The **Driving Force Observer (DFO)**, pioneered by Hori's laboratory at the University of Tokyo, is the most widely adopted solution. It treats F_x·R as a disturbance torque and uses a first-order low-pass filter in a feedback configuration: F̂_x = (1/R)·[T_motor − J_w·dω/dt], filtered through an LPF with time constant τ that trades noise rejection against response speed. This approach requires no tire model, works at any slip level, and runs on minimal computational hardware. Hori (2004) demonstrated this on the "UOT Electric March" test vehicle, establishing the paradigm that electric motor torque knowledge replaces the need for dedicated force sensors.

More robust variants include the **Luenberger + High-Order Sliding Mode Observer** combination (Chen et al., 2018), which couples the motor electromagnetic model with wheel mechanics into an Electric Driving Wheel Model (EDWM). The system undergoes exact feedback linearization, after which an adaptive HSMO estimates Fx with formal Lyapunov stability guarantees. A Kalman filter post-processes the output for noise optimality. FS Team Tallinn implemented a practical variant — a **discrete-time Luenberger observer** with pole placement (poles at [−50, −100, ..., −450]) running at 200 Hz on their microcontroller, validated on their 169 kg, 2.1-second 0–100 km/h car.

A **PID observer** approach (Wang et al., 2025) incorporates motor current as an input rather than just torque command, using the full electromechanical coupling (resistance, inductance, back-EMF constant) for improved accuracy. **High-gain observers** using tire relaxation length dynamics (Ḟ_x = (V/σ)(f(λ) − F_x)) provide another pathway that requires only wheel speed measurement.

---

## Lateral force estimation demands vehicle-level models

Unlike Fx, **lateral tire force does not appear in the wheel rotational dynamics** — it acts perpendicular to the wheel's spin axis and produces no torque about it. Fy estimation therefore requires vehicle-level models combining lateral acceleration, yaw rate, and longitudinal dynamics.

The bicycle (single-track) model provides the simplest framework: m·(v̇_y + v_x·ψ̇) = F_yf + F_yr for lateral dynamics and I_z·ψ̈ = l_f·F_yf − l_r·F_yr for yaw. With IMU-measured lateral acceleration and yaw rate, these two equations yield per-axle lateral forces but not individual wheel forces (which require load transfer assumptions). The four-wheel (double-track) model resolves individual forces but introduces more unknowns, typically requiring tire models to close the system.

The **Extended Kalman Filter** is the most established approach. Ray (1997) implemented a 17th-order EKF with a 5-DOF bicycle model and third-order random walk tire force model, estimating per-axle lateral forces from yaw rate, wheel speeds, and accelerations. Doumiati et al. (2011) compared EKF and UKF implementations using a four-wheel model with Dugoff tires, finding **UKF showed superior convergence**, particularly near tire saturation — a finding consistently replicated across the literature.

The **Unscented Kalman Filter** avoids Jacobian computation entirely, propagating sigma points through the full nonlinear model. Hamann et al. (2014) demonstrated a UKF with state vector [v_x, v_y, ψ̇, ω_f, ω_r, F_xf, F_xr, F_yf, F_yr] using a random walk tire force model (F̈ = white noise), eliminating the need for any tire parameter identification. This "model-free" approach to tire forces, combined with the UKF's ability to handle nonlinearities, makes it particularly attractive for racing applications where tire behavior varies rapidly with temperature and wear. Alshawi, Sorniotti et al. (2024) extended this with **adaptive covariance matrices** on both process and measurement noise, demonstrating robustness under variable friction conditions.

The **Cubature Kalman Filter** uses third-order spherical-radial cubature rules to solve the nonlinear-Gaussian integration problem, avoiding both EKF's Taylor truncation and UKF's potential positive-definiteness issues. Research with 7-DOF models and Magic Formula tires shows CKF excels in strongly nonlinear regions (high slip, low friction) where EKF and UKF accuracy degrades.

A particularly elegant approach for 4WID EVs exploits the **cascaded estimation architecture**: since all four Fx values are known from wheel-level DFOs, the yaw moment from longitudinal forces can be computed directly. A disturbance observer on the yaw dynamics then treats the lateral force contribution as an unknown disturbance, enabling Fy estimation without any tire model. This three-step cascade — yaw moment estimation, axle force summation via least squares, individual force allocation via heuristics — leverages the 4WID's unique advantage of known per-wheel torque.

**Sliding mode observers** offer formal robustness guarantees against model uncertainty. Razmjooei et al. (2025) showed a second-order sliding mode observer outperforming both EKF and SDRE filters for lateral force estimation. The 7-DOF SMO with UniTire model (SAE 2014-01-0865) adds system damping to accelerate convergence while suppressing chattering. The Viehweger et al. (2021) benchmark study — comparing EKF with adaptive cornering stiffness, EKF with neural network tire model, super-twisting sliding mode, and kinematic-model EKF on identical experimental data — represents the most rigorous published comparison of these methods.

---

## Friction and slip estimation close the control loop

**Tire-road friction coefficient (μ)** estimation remains the hardest problem because μ is fundamentally unobservable when tires operate in the linear region — the force-slip curve's initial slope conflates friction with tire stiffness. Gustafsson's seminal 1997 work established the slip-slope method: at low slip, the μ-slip curve is approximately linear, and its slope correlates with maximum available μ, but the relationship is indirect and requires calibration. Maximum friction force methods (Lee, Hedrick & Yi, 2004; Rajamani et al., 2012) monitor dμ/dλ, detecting the friction limit when this derivative approaches zero — practical for racing where tires frequently approach saturation.

The 4WID architecture enables a unique **torque injection method**: applying equal and opposite torque perturbations to left/right wheels produces differential slip for μ probing without affecting vehicle trajectory, cycling at 50–100 ms periods. Brush model-based estimation (Singh & Taheri, 2014) fits stored force-slip data points via Levenberg-Marquardt optimization, updating μ only when sufficient nonlinearity is detected. EKF/UKF-based μ estimation typically augments the state vector with μ as an additional state, requiring sufficient tire excitation for observability. The interacting multiple model (IMM) approach by Ahn, Peng & Tseng (2011) probabilistically switches between multiple estimator models to cover different excitation levels.

**Slip ratio** estimation (λ = (Rω − V_x)/max(Rω, V_x)) requires knowing vehicle longitudinal velocity V_x, which cannot be directly measured with standard sensors. For 4WID EVs where all wheels are driven, the traditional "non-driven wheel speed averaging" method fails entirely. The recommended approach fuses wheel speeds, IMU accelerometer integration (for high-rate prediction), and GPS velocity (for drift-free low-rate correction) in an EKF or complementary filter. FST Lisboa validated this architecture on their FST06e car, achieving accurate velocity estimation at 100 Hz using a low-cost IMU and 25 Hz GPS, validated against differential GPS ground truth.

**Slip angle** estimation is fundamentally a lateral velocity (V_y) estimation problem, since tire slip angles depend on V_y (e.g., α_f = δ − arctan((V_y + l_f·r)/V_x)). The kinematic approach — integrating V̇_y = a_y − V_x·r — works during transients but drifts in steady state. The dynamic approach — using a bicycle model EKF with cornering stiffness parameters — works in steady state but can diverge during rapid transients if the tire model is inaccurate. **Combined kinematic-dynamic observers** weight between these two approaches adaptively: β_est = w·β_kinematic + (1−w)·β_dynamic, where w increases during transient maneuvers (large |ṙ| or |δ̇|) and decreases during steady cornering. The cross-combined UKF (Meccanica, 2021) represents the state of the art, with the Sorniotti group's adaptive UKF (2024) adding friction-condition robustness. Expensive optical sensors like Kistler Correvit (€15,000–30,000+, 0.1° slip angle accuracy) serve as ground truth for validation but are impractical for competition.

---

## Formula Student teams favor pragmatic estimation architectures

Published FSE implementations reveal a strong preference for **proven, simple methods over theoretically optimal ones** — driven by annual team turnover, limited testing time (2–4 weeks before competition), and embedded hardware constraints.

**AMZ Racing (ETH Zurich)** uses IMU-derived accelerations for load transfer computation and a UKF for state estimation in their driverless system (Kabzan et al., 2019, arXiv:1905.05150), with a constant-acceleration process model fusing IMU and linear tire force models. Their yaw rate from the gyroscope is described as "the most important value for our torque vectoring system." They reported electromagnetic interference from 200 A motor currents corrupting IMU signals — a practical challenge rarely mentioned in academic papers.

**FS Team Tallinn (TalTech)**, ranked world #1 in FSE in 2024, implemented a **discrete-time Luenberger observer** for per-wheel tractive force estimation feeding a sliding mode controller for slip regulation. They explicitly chose "classical control solutions that do not require significant computing capacity" due to financial and weight constraints, running at 200 Hz on a standard microcontroller. Their work, validated on a car achieving 3G lateral acceleration, acknowledged the limitation of straight-line-only design and recommended incorporating lateral dynamics in future iterations.

**FST Lisboa (IST)** published the most complete validated sideslip estimation pipeline: an Attitude Complementary Filter for heading, a Position Complementary Filter fusing GPS and accelerometer for velocity, and an **EKF-based vehicle lateral estimator** using a Burckhardt tire model. Validated against differential GPS at 100 Hz, their nonlinear estimator significantly outperformed the linear version. They subsequently tested PID and LQR torque vectoring controllers on the real car.

**Mikula et al. (2018, IEEE CDC)** demonstrated the most advanced published FSE estimation-control combination: a **UKF feeding both LTV-MPC and NMPC** torque vectoring controllers on a two-track vehicle model with nonlinear tires. The UKF was chosen because it "enabled to utilize the full nonlinear vehicle model while sustaining lower computational effort compared to the EKF" — a counterintuitive finding explained by the UKF avoiding expensive Jacobian derivation for complex models. **UninaCorse** (University of Naples) reported a **9% skidpad lap time improvement** from their three-layer torque vectoring system using a double-track model with Pacejka tires.

---

## Each estimated parameter serves a specific layer of the torque vectoring stack

The three-layer torque vectoring architecture established by De Novellis, Sorniotti, and Gruber through the EU FP7 E-VECTOORC project provides the canonical framework. Each estimated parameter enters at a specific point:

The **reference generator** requires vehicle speed (V_x, estimated via wheel speed/IMU/GPS fusion), steering angle (δ, measured), and friction estimate (μ̂) to compute the desired yaw rate: r_ref = (V_x·δ)/(L·(1 + K_us·V_x²)), limited by the achievable lateral acceleration |r_ref| ≤ μ̂·g/V_x. Without accurate μ estimation, the reference can demand physically impossible yaw rates, causing the controller to fight the tire physics. Advanced implementations (De Novellis et al., 2014) use full nonlinear understeer characteristic maps r_ref(δ, V_x) supporting different driving modes.

The **high-level controller** computes the required yaw moment M_z from the error between measured and reference yaw rate (e_r = r_ref − r) and sideslip angle (e_β = β_ref − β̂). This layer needs real-time yaw rate (measured by gyroscope), estimated sideslip angle (from the kinematic-dynamic observer), and optionally estimated lateral forces for model-based controllers. Controller options range from PID on yaw rate error (AMZ, FST Lisboa — simplest and most robust to annual team turnover) through LQR (FST Lisboa), sliding mode (Goggia, Sorniotti et al., 2015), to MPC (Mikula et al., 2018). The output is a total longitudinal force demand F_x,total and a desired corrective yaw moment M_z,desired.

The **control allocator** is where Fz estimation matters most. It distributes F_x,total and M_z,desired to four individual wheel torques subject to friction circle constraints: F_x,i² + F_y,i² ≤ (μ̂·F̂_z,i)² at each wheel, plus motor torque-speed limits |T_i| ≤ T_max(ω_i). Quadratic programming minimizes a cost function (tire slip power losses, or deviation from equal tire utilization) subject to these constraints. The friction circle radius **scales directly with Fz** — underestimating Fz makes the allocator too conservative (leaving grip unused), while overestimating it risks commanding forces beyond the tire's capability, causing saturation and potential instability. Alternative allocation methods include pseudo-inverse weighting proportional to 1/Fz² (equalizing tire utilization), axle saturation proportional distribution, and rule-based approaches using load transfer for grip prioritization.

The following table summarizes the parameter-to-layer mapping:

| Estimated parameter | Reference generator | High-level controller | Control allocator |
|---|---|---|---|
| Vehicle speed (V_x) | ✓ (yaw rate reference) | ✓ (model dynamics) | — |
| Sideslip angle (β) | — | ✓ (error signal) | — |
| Yaw rate (r) | — | ✓ (error signal) | — |
| Vertical load (Fz,i) | — | — | ✓ (friction circle sizing) |
| Longitudinal force (Fx,i) | — | Optional (feedforward) | ✓ (current tire state) |
| Lateral force (Fy,i) | — | Optional (model-based) | ✓ (friction circle usage) |
| Friction coefficient (μ) | ✓ (yaw rate limiting) | — | ✓ (friction circle scaling) |
| Slip ratio (λ_i) | — | — | ✓ (traction control) |
| Slip angle (α_i) | — | — | ✓ (tire model evaluation) |

---

## Practical sensor suite and observer selection for FSE

The minimum viable sensor set for a competitive FSE estimation system consists of four components: a **6-axis IMU** (±16g accelerometer, ±2000°/s gyroscope, >200 Hz — options range from €10 GY-80 modules to €2000+ SBG Ellipse-N units), **4× wheel speed** from motor encoder/resolver signals via CAN bus (typically thousands of counts per revolution), a **steering angle sensor** (rotary encoder or potentiometer, 0.1° resolution), and **motor torque feedback** from each inverter's q-axis current measurement. Adding a GPS receiver (even a basic 10 Hz u-blox at ~€50) provides drift-free velocity correction that dramatically improves V_x and V_y estimation.

For observer selection, the evidence from published FSE implementations is clear. The EKF remains the practical workhorse — well-supported by MATLAB/Simulink toolchains, running at ~100–500 μs per step on ARM Cortex-M4 hardware, and requiring moderate tuning effort (Q and R covariance matrices). The UKF is preferred when nonlinear model fidelity matters (combined slip, tire saturation in racing), running at ~1–5 ms on Cortex-M7 or dSPACE hardware. Luenberger observers handle per-wheel force estimation where simplicity and minimal computation are paramount (~10 μs per step). Sliding mode observers offer the strongest robustness guarantees against model uncertainty but require chattering mitigation and have seen limited on-car FSE deployment.

The most critical uncertain parameter across all observer types is **tire cornering stiffness**, which varies with vertical load, temperature, wear, and surface condition. Adaptive approaches — augmenting the EKF state vector to estimate cornering stiffness online, or using recursive least squares identification — are increasingly standard in competitive implementations.

## Conclusion

The estimation problem for 4WID Formula Student EVs is fundamentally reshaped by the availability of precise per-wheel motor torque, which enables a cascaded architecture: wheel-level disturbance/Luenberger observers extract longitudinal forces directly, while vehicle-level EKF/UKF estimators use these as known inputs to reconstruct lateral forces, sideslip, and friction. Vertical load estimation via quasi-static load transfer models using IMU accelerations is both the simplest and most universally adopted approach, though suspension-sensor-based and EKF-based methods offer improved transient accuracy. The key insight from real FSE implementations is that **estimation system maturity matters more than theoretical optimality** — FS Team Tallinn won with Luenberger observers, AMZ dominates with acceleration-based load transfer, and FST Lisboa validated their EKF on a €10 IMU. The friction coefficient remains the least observable parameter, practically estimated only when tires approach saturation — which, in racing, is precisely when it matters most. For teams beginning development, the recommended progression is: (1) quasi-static Fz + DFO for Fx using motor torque, (2) EKF for sideslip/velocity fusing IMU + GPS + wheel speeds, (3) adaptive tire parameter estimation and μ monitoring as the system matures through testing.
