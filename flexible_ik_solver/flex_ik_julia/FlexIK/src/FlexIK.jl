# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

module FlexIK

import LinearAlgebra: pinv, I, svd, det, norm, Diagonal, argmin, rank, opnorm

"""
    struct InverseParams

The `InverseParams` struct represents the parameters used for matrix inversion calculations.

# Fields
- `tolerance::Float64`: The tolerance value used to check the numerical conditioning of the input matrix. Default is `1e-1`.
- `damping_coeff::Float64`: The maximum damping coefficient applied to the singular directions. Default is `1e-1`.

"""
@kwdef mutable struct InverseParams
    tolerance::Float64 = 1e-1
    damping_coeff::Float64 = 1e-1
end

"""
    srinv(inv_params::InverseParams, A)

Compute the inverse of a matrix using the Singular Value Decomposition (SVD) method with extended regularization.
This implementation is based on SRINV_EXT in `workspaces/bdai/src/flexible_ik_solver/flex_ik_py/inverse_solver.py`

# Arguments
- `inv_params`: The inverse solver param struct.
- `A`: The input matrix.

# Returns
The inverse of the input matrix `A`.
"""
function srinv(inv_params::InverseParams, A)
    # Get the SVD and determinant of the matrix
    U, S, _ = svd(A, full=true)
    AAT = A * A'

    # Compute determinant and lambda value.
    # Note that lamdba_val is a scalar damping constant for the entire matrix,
    # which is used to numerically stabilize the matrix when the determinant is below threshold.
    threshold = inv_params.tolerance^2
    determinant = det(AAT)
    lambda_val = determinant < threshold ? (1.0 - (determinant / threshold)^2) * threshold : 0.0

    # Check if the singular values are below the tolerance,
    # and compute the damping coefficients accordingly.
    # This damping gets applied to only singular directions.
    tempMat = AAT + lambda_val * I
    below_tolerance = S .< inv_params.tolerance
    u_sum_scaled = zeros(size(A, 1), size(A, 1))
    if any(below_tolerance)
        beta = ifelse.(below_tolerance, inv_params.damping_coeff * (1.0 .- (S ./ inv_params.tolerance).^2), 0)
        U_reduced = U[:, below_tolerance]
        u_sum_scaled .= U_reduced * Diagonal(beta[below_tolerance]) * U_reduced'
        tempMat .+= u_sum_scaled
    end

    # Reconstruct the inverse
    return A' / tempMat
end

"""
    find_scale_factor(low, upp, a)

Given a lower margin `low`, an upper margin `upp`, and a value `a`,
this function calculates the feasible scale factor considering the margins.
The function returns a tuple `(sUpp, sLow)` where `sUpp` is the feasible scale factor
considering the upper margin and `sLow` is the feasible scale factor considering the lower margin.

# Arguments
- `low`: The lower margin value.
- `upp`: The upper margin value.
- `a`: The value for which the scale factor is calculated.

# Returns
A tuple `(sUpp, sLow)` where `sUpp` is the feasible scale factor considering the upper margin
and `sLow` is the feasible scale factor considering the lower margin.
"""
function find_scale_factor(low, upp, a)
    if low > upp
        @error("The lower margin ($low) must be <= the upper margin ($upp).")
    end

    sUpp = 1.0  # the feasible scale factor considering upper margin
    sLow = 0.0  # the feasible scale factor considering lower margin

    THRES_TOO_SMALL = 1e-10  # Adjust the threshold as needed
    THRES_TOO_LARGE = 1e+10  # Adjust the threshold as needed

    if THRES_TOO_SMALL < abs(a) < THRES_TOO_LARGE
        if a < 0.0 && low < 0.0 && a <= upp
            sUpp = low / a
            sLow = upp / a
        elseif a > 0.0 && upp > 0.0 && a >= low
            sUpp = upp / a
            sLow = low / a
        end
    end

    return (min(1.0, sUpp), max(0.0, sLow))
end

"""
    calculateBoxConstraints(margin_low, margin_upp, vel_max, acc_max, ts)

Calculate the box constraints for a given motion profile.

# Arguments
- `margin_low`: The lower margin of the motion profile.
- `margin_upp`: The upper margin of the motion profile.
- `vel_max`: The maximum velocity allowed.
- `acc_max`: The maximum acceleration allowed.
- `ts`: The time step in seconds.

# Returns
- `lim_low`: The lower limit of the box constraints.
- `lim_upp`: The upper limit of the box constraints.
"""
function calculateBoxConstraints(margin_low, margin_upp, vel_max, acc_max, ts)
    margin_low = min.(margin_low, 0)
    margin_upp = max.(margin_upp, 0)

    lim_low = max.(margin_low ./ ts, -vel_max,
        -sqrt.(2 .* acc_max .* abs.(margin_low))
    )
    lim_upp = min.(margin_upp ./ ts, vel_max,
        sqrt.(2 .* acc_max .* abs.(margin_upp))
    )
    return lim_low, lim_upp
end

"""
    compute_joint_constraints(q, q_min, q_max, qd_max, qdd_max, time_step)

Compute the joint constraints for a given set of joint variables.

# Arguments
- `q`: Current joint configuration.
- `q_min`: Vector of minimum joint position limits.
- `q_max`: Vector of maximum joint position limits.
- `qd_max`: Vector of maximum joint velocity limits.
- `qdd_max`: Vector of maximum joint acceleration limits.
- `time_step`: Time step for constraint calculation in seconds.

# Returns
- `C`: Identity matrix of size `nq`, where `nq` is the length of `q`.
- `lim_low`: Vector of lower joint constraints.
- `lim_upp`: Vector of upper joint constraints.
"""
function compute_joint_constraints(
    q,
    q_min,
    q_max,
    qd_max,
    qdd_max,
    time_step
    )
    if any(length(var) != length(q) for var in [q_min, q_max, qd_max, qdd_max])
        error_msg = "Dimension mismatch: (q ($(length(q))), q_min ($(length(q_min))), " *
                    "q_max ($(length(q_max))), qd_max ($(length(qd_max))), qdd_max ($(length(qdd_max))))."
        @error error_msg
    end

    MARGIN_LIMIT = 1e-4

    q_min = copy(q_min)
    q_max = copy(q_max)
    qd_max = copy(qd_max)
    qdd_max = copy(qdd_max)

    # Apply small margin to the limits to consider numerical issues
    q_min .+= MARGIN_LIMIT
    q_max .-= MARGIN_LIMIT
    qd_max .-= MARGIN_LIMIT
    qdd_max .-= MARGIN_LIMIT

    nq = length(q)
    C = I(nq)
    margin_low = q_min - q # negative margin
    margin_upp = q_max - q # positive margin
    lim_low, lim_upp = calculateBoxConstraints(margin_low, margin_upp, qd_max, qdd_max, time_step)

    return C, lim_low, lim_upp

end

"""
    compute_floatingbase_constraints(error, bounds_low, bounds_upp, vel_max, acc_max, time_step)

Compute the floating base constraints based on the given error, bounds, velocity limits, acceleration limits, and time step.

# Arguments
- `error`: Vector of current pose error values.
- `bounds_low`: Vector of lower bounds in the pose.
- `bounds_upp`: Vector of upper bounds in the pose.
- `vel_max`: Vector of maximum velocity limits.
- `acc_max`: Vector of maximum acceleration limits.
- `time_step`: Time step for constraint calculation in seconds.

# Returns
- `C`: Identity matrix of size equal to the length of the error vector.
- `lim_low`: Vector of lower limits after applying margins.
- `lim_upp`: Vector of upper limits after applying margins.
"""
function compute_floatingbase_constraints(
    error,
    bounds_low,
    bounds_upp,
    vel_max,
    acc_max,
    time_step
    )
    if !(length(error) == length(bounds_low) == length(bounds_upp) == length(vel_max) == length(acc_max))
        @error("Dimension mismatch: (error, bounds_low, bounds_upp, vel_max, acc_max).")
    end

    MARGIN_LIMIT = 1e-4

    bounds_low = copy(bounds_low)
    bounds_upp = copy(bounds_upp)
    vel_max = copy(vel_max)
    acc_max = copy(acc_max)

    # Apply small margin to the limits to consider numerical issues
    bounds_low .+= MARGIN_LIMIT
    bounds_upp .-= MARGIN_LIMIT
    vel_max .-= MARGIN_LIMIT
    acc_max .-= MARGIN_LIMIT

    C = I(length(error))

    margin_low = bounds_low + error # negative margin
    margin_upp = bounds_upp + error # positive margin
    lim_low, lim_upp = calculateBoxConstraints(margin_low, margin_upp, vel_max, acc_max, time_step)

    return C, lim_low, lim_upp
end

"""
    compute_wholebody_constraints(error, bounds_low, bounds_upp, vel_max, acc_max, q, q_min, q_max, qd_max, qdd_max, time_step)

Compute the whole-body constraints for a given set of parameters.

# Arguments
- `error`: The error vector.
- `bounds_low`: The lower bounds vector.
- `bounds_upp`: The upper bounds vector.
- `vel_max`: The maximum velocity vector.
- `acc_max`: The maximum acceleration vector.
- `q`: The joint positions vector.
- `q_min`: The minimum joint positions vector.
- `q_max`: The maximum joint positions vector.
- `qd_max`: The maximum joint velocities vector.
- `qdd_max`: The maximum joint accelerations vector.
- `time_step`: The time step.

# Returns
- `C`: The constraint matrix.
- `lim_low`: The lower bounds vector.
- `lim_upp`: The upper bounds vector.
"""
function compute_wholebody_constraints(
        error,
        bounds_low,
        bounds_upp,
        vel_max,
        acc_max,
        q,
        q_min,
        q_max,
        qd_max,
        qdd_max,
        time_step
    )
    C_base, lim_low_base, lim_upp_base = compute_floatingbase_constraints(
        error, bounds_low, bounds_upp, vel_max, acc_max, time_step
    )

    C_jnts, lim_low_jnts, lim_upp_jnts = compute_joint_constraints(
        q, q_min, q_max, qd_max, qdd_max, time_step
    )

    # Create an zero matrix with the appropriate size
    C = zeros(size(C_base) .+ size(C_jnts))

    # Copy the base constraints and joint constraints to the appropriate positions in C
    C[1:size(C_base, 1), 1:size(C_base, 2)] .= C_base
    C[size(C_base, 1) + 1:end, size(C_base, 2) + 1:end] .= C_jnts

    # Concatenate the lower and upper bounds
    lim_low = vcat(lim_low_base, lim_low_jnts)
    lim_upp = vcat(lim_upp_base, lim_upp_jnts)

    return C, lim_low, lim_upp
end

"""
    solve_joints_vel_ik(q, q_min, q_max, qd_max, qdd_max, time_step, taskGoalData, taskJacobianData, inv_params)

Solves the inverse kinematics problem for joint velocities given the current joint configuration
and the limits for the input task data.

# Arguments
- `q`: Current joint configuration.
- `q_min`: Minimum joint limits.
- `q_max`: Maximum joint limits.
- `qd_max`: Maximum joint velocities.
- `qdd_max`: Maximum joint accelerations.
- `time_step`: Time step for the inv_params.
- `taskGoalData`: Task goal data.
- `taskJacobianData`: Task Jacobian data.
- `inv_params`: The inverse solver parameters.

# Returns
- Joint velocities that satisfy the inverse kinematics problem.

"""
function solve_joints_vel_ik(
    q,
    q_min,
    q_max,
    qd_max,
    qdd_max,
    time_step,
    taskGoalData,
    taskJacobianData,
    inv_params::InverseParams
    )
    C, limLow, limUpp = compute_joint_constraints(
        q,
        q_min,
        q_max,
        qd_max,
        qdd_max,
        time_step
    )

    return solve_vel_ik(taskGoalData, taskJacobianData, C, limLow, limUpp, inv_params)
end

"""
    solve_wholebody_vel_ik(error, bounds_low, bounds_upp, vel_max, acc_max, q, q_min, q_max, qd_max, qdd_max, time_step, taskGoalData, taskJacobianData, inv_params)

Solves the whole-body velocity inverse kinematics problem.

# Arguments
- `error`: The error vector representing the floating-base pose error.
- `bounds_low`: The lower bounds of the floating-base pose.
- `bounds_upp`: The upper bounds of the floating-base pose.
- `vel_max`: The maximum joint velocity limits.
- `acc_max`: The maximum joint acceleration limits.
- `q`: The current joint positions.
- `q_min`: The lower bounds of the joint positions.
- `q_max`: The upper bounds of the joint positions.
- `qd_max`: The maximum joint velocity limits.
- `qdd_max`: The maximum joint acceleration limits.
- `time_step`: The time step in seconds.
- `taskGoalData`: The task goal data.
- `taskJacobianData`: The task Jacobian data.
- `inv_params`: The inverse solver parameters.

# Returns
The joint velocities that satisfy the whole-body velocity inverse kinematics problem.
"""
function solve_wholebody_vel_ik(
        error,
        bounds_low,
        bounds_upp,
        vel_max,
        acc_max,
        q,
        q_min,
        q_max,
        qd_max,
        qdd_max,
        time_step,
        taskGoalData,
        taskJacobianData,
        inv_params::InverseParams
    )
    C, limLow, limUpp = compute_wholebody_constraints(
        error,
        bounds_low,
        bounds_upp,
        vel_max,
        acc_max,
        q,
        q_min,
        q_max,
        qd_max,
        qdd_max,
        time_step
    )

    return solve_vel_ik(taskGoalData, taskJacobianData, C, limLow, limUpp, inv_params)
end


"""
    solve_vel_ik(taskGoalData, taskJacobianData, C, limLow, limUpp, inv_params)

Solves the velocity inverse kinematics problem for a given set of tasks.

## Arguments
- `taskGoalData`: An array of task goal data.
- `taskJacobianData`: An array of task Jacobian data.
- `C`: The constraint matrix.
- `limLow`: The lower limits for the joint positions.
- `limUpp`: The upper limits for the joint positions.
- `inv_params`: The inverse solver parameters.

## Returns
- `dqi`: The joint velocity solution.
- `sData`: An array of scale factors for each task.
- `exitCode`: The exit code indicating the status of the solution.

The function iteratively solves for the joint velocities that satisfy the task goals and constraints. It uses different inverse solver methods based on the selected_inv_method parameter.

"""
function solve_vel_ik(taskGoalData, taskJacobianData, C, limLow, limUpp, inv_params::InverseParams)

    if !(length(taskJacobianData) == length(taskGoalData))
        @error("Dimension mismatch: (jacobianData, dxGoalData).")
    end

    tol = 1e-6
    tol_tight = 1e-10
    THRES_TOO_LARGE = 1e10
    max_iter=20

    nJnt = size(C, 2)
    k = size(C, 1) - nJnt

    n_tasks = length(taskJacobianData)
    sData = zeros(n_tasks)

    Pi = I(nJnt)
    Sact = zeros(nJnt + k, nJnt + k)
    dqi = zeros(nJnt)
    w = zeros(nJnt + k)
    Cact = zeros(nJnt + k, nJnt)

    # Check if C matrix is an identity matrix
    C = C == I(nJnt) ? 1 : C

    # Preallocate memory for variables
    dqiPrev = similar(dqi)
    wStar = similar(w)
    z = similar(w)
    a = similar(w)
    b = similar(w)
    marginL = similar(w)
    marginU = similar(w)
    sMax = similar(w)
    PiHatStar = zeros(nJnt, nJnt + k)
    PiHat = similar(PiHatStar)

    exitCode = :SUCCESS

    for iTask in 1:n_tasks
        Ji = taskJacobianData[iTask]
        dxGoali = taskGoalData[iTask]
        ndxGoal = size(Ji, 1)

        # update variables for previous projection matrix and solution
        PiPrev = Pi
        dqiPrev .= dqi

        # initialize variables for i-th task
        PiBar = PiPrev
        si = 1
        siStar = 0

        limit_exceeded = true
        cntLoop = 0

        SactStar = Sact
        wStar .= w
        PiBarStar = zero(PiBar)
        fill!(PiHatStar, 0)

        if norm(dxGoali,1) < tol_tight
            dqi .= dqiPrev
            limit_exceeded = false
        end

        # Pre-compute terms for optimization
        inv_Ji_PiBar = srinv(inv_params, Ji * PiBar)
        Ji_dqiPrev = Ji * dqiPrev
        Cact_PiPrev = iszero(Sact) ? zeros(nJnt + k, nJnt) : Cact * PiPrev
        Cact_dqiPrev = iszero(Sact) ? zeros(nJnt + k, 1) : Cact * dqiPrev

        while limit_exceeded
            limit_exceeded = false

            # update PiHat
            PiHat .= (I - inv_Ji_PiBar * Ji) * pinv(Cact_PiPrev, tol)

            # compute a solution without task scale factor
            dqi .= dqiPrev + inv_Ji_PiBar * (dxGoali - Ji_dqiPrev) + PiHat * (w - Cact_dqiPrev)

            # check whether the solution violates the limits
            z .= C * dqi
            if any(z .< (limLow .- tol)) || any(z .> (limUpp .+ tol))
                limit_exceeded = true
            end

            # compute goal velocity projected
            a .= C * inv_Ji_PiBar * dxGoali
            b .= z - a

            a_norm = norm(a)
            if a_norm < tol
                # if the projected goal velocity is close to zero, set scale factor to one (feasible)
                si = 1.0
                mclIdx = 1
            elseif a_norm > THRES_TOO_LARGE  # Adjust the threshold as needed
                @warn "Projected goal for task $iTask is too small (inverse solver output might be invalid)."

                si = 0.0
                mclIdx = 1
                exitCode = :INVERSE_SOLVER_UNSTABLE
            else
                marginL .= limLow - b
                marginU .= limUpp - b

                for iLim in 1:nJnt+k
                    if Sact[iLim, iLim] == 1
                        sMax[iLim] = Inf
                    else
                        sMax[iLim], _ = find_scale_factor(marginL[iLim], marginU[iLim], a[iLim])
                    end
                end

                # most critical limit index
                mclIdx = argmin(sMax)
                si = sMax[mclIdx]

            end

            if isinf(si)
                si = 0.0
            end

            # Do the following only if the task is feasible and the scale factor calculated is correct
            if (iTask == 1 || si > 0) && cntLoop < max_iter
                scaledDqi = dqiPrev + inv_Ji_PiBar * (si * dxGoali - Ji_dqiPrev) + PiHat * (w - Cact_dqiPrev)

                z .= C * scaledDqi

                limitSatisfied = !(any(z .< (limLow .- tol)) || any(z .> (limUpp .+ tol)))

                if si > siStar && limitSatisfied
                    siStar = si
                    SactStar = Sact
                    wStar .= w
                    PiBarStar = PiBar
                    PiHatStar .= PiHat
                end

                Sact[mclIdx, mclIdx] = 1
                Cact = Sact * C
                w[mclIdx, 1] = min(max(limLow[mclIdx], z[mclIdx]), limUpp[mclIdx])

                Cact_PiPrev = Cact * PiPrev

                PiBar = PiPrev - pinv(Cact_PiPrev, tol) * (Cact_PiPrev)

                taskRank = opnorm(PiBar, 1) < tol_tight ? 0 : rank(Ji * PiBar, tol)
                if taskRank < ndxGoal
                    si = siStar
                    Sact = SactStar
                    Cact = Sact * C
                    w .= wStar
                    PiBar = PiBarStar
                    PiHat .= PiHatStar

                    dqi .= dqiPrev + srinv(inv_params, Ji * PiBar) * (si * dxGoali - Ji_dqiPrev) + PiHat * (w - Cact * dqiPrev)

                    limit_exceeded = false
                end

                if cntLoop == max_iter
                    @error "the maximum number of iterations has been reached!"
                    exitCode = :MAX_ITERATION_REACHED  # Define the exitCode variable appropriately
                    return dqi, sData, exitCode
                end
            else  # If the current task is infeasible
                si = 0
                dqi .= dqiPrev
                limit_exceeded = false

                if cntLoop == max_iter
                    @error "the maximum number of iterations has been reached!"
                    exitCode = :MAX_ITERATION_REACHED  # Define the exitCode variable appropriately
                    return dqi, sData, exitCode
                end
            end

            cntLoop += 1

            if si > 0.0
                Pi = PiPrev - pinv(Ji * PiPrev, tol) * Ji * PiPrev
            end

            # Update the pre-computed terms
            inv_Ji_PiBar .= srinv(inv_params, Ji * PiBar)
            Cact_PiPrev .= Cact * PiPrev
            Cact_dqiPrev .= Cact * dqiPrev
        end

        sData[iTask] = si
    end

    if sum(sData) > tol && (any(C * dqi .< (limLow .- tol)) || any(C * dqi .> (limUpp .+ tol)))
        @error "The solution violated the constraints!"
        exitCode = :LIMIT_VIOLATION
    end

    return dqi, sData, exitCode
end





end # module FlexIK
