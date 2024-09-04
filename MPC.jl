using JuMP
using Gurobi
import GLPK
import LinearAlgebra
import Random
using Plots
using Latexify
using LaTeXStrings


println("All libraries imported\n")

# DIMENSIONS
# Time horizon
DK = 20;

# Workstation
DW = 3;

# Resources
DR = 2;

# Tasks
DT = 5;

# PARAMETERS
# PART 1 --- TASKS

# Time required by workstations to process specific task (DW*DT)
T_ij = [4 5 8 5 2;
        5 6 12 3 6;
        3 4 5 3 5];

# Tasks deadlines times (finish stricltly less than k specified) (1*DT) 
T_j = [20, 20, 20, 20, 20];

# PART 2 --- RESOURCES
# Workstation capacity (DW*1)
W_i = [1, 3, 1];

# Workstations maximal storage inventory for each kind of resource (DW*DR)
W_ir = [50 50;
        50 50;
        50 50];

# Required amount of resources for each tasks (DT*DR)
T_jr = [13 12;
        6 7;
        4 3;
        2 4;
        3 5];
# Resources level needed for each workstation (DW*DR)
R_ir = [[10 10;
         20 20;
         5  5] for k in 1:DK];

# Task precedence matrix (1 if i-th must be exected before j-th) (DT*DT)
T_jj = [0 1 0 0 1;
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 0 1;
        0 0 0 0 0];


# Initialization of Resources available (1*DR)
R_r = [100, 200];






# DISTURBANCES PARAMETERS : Delays
# Id of delayed Task:
i = 3                # workstation
j = 2                # task
# Delay Duration
Delta = [8,9,10,11,12,13]






# MODEL CREATION
m = Model(Gurobi.Optimizer);
println("Model has been succesfully created\n")

# --- TASKS ---
# --- STATE VARIABLES (TASKS)
# Duration
@variable(m, d[1:DW, 1:DT, 0:DK] >= 0, start=0, Int);

# Started task
@variable(m, s[1:DW, 1:DT, 0:DK], start=0, Bin);

# Execution states in workstation w at time k
@variable(m, e[1:DW, 1:DT, 0:DK], start=0, Bin);

# Finished states at time k
@variable(m, f[1:DW, 1:DT, 0:DK], start=0, Bin);

# Occupancy level
@variable(m, o[1:DW, 0:DK] >= 0, start=0, Int);

# --- CONTROL VARIABLES (TASKS)
# Task assignment (control)
@variable(m, at[1:DW, 1:DT, 0:DK], start=0, Bin)

# --- SLACK VARIABLES (TASKS)
# Needed for dynamic duration update
@variable(m, slack[1:DW, 1:DT, 0:DK], Bin)

# Needed for time precedence
@variable(m, slack2[1:DW, 1:DT, 0:DK], Bin)


# --- RESOURCES ---
# --- STATE VARIABLES (RESOURCES)
# Available resources in the whole assembly line
@variable(m, R[1:DR, 0:DK] >= 0, Int);

# Resources allocated in a workstation 
@variable(m, 0 <= r[i in 1:DW, r in 1:DR, 0:DK] <= W_ir[i,r], Int);

# --- CONTROL VARIABLES (RESOURCES)
# Resources assignment (control)
@variable(m, ar[1:DW, 1:DT, 1:DR, 0:DK], start=0, Bin)

# --- SLACK VARIABLES (RESOURCES)
# Needed for available resources update
@variable(m, slack3[1:DR, 0:DK] >=0, Int)

# Needed for resources update (move from R to r when task starts)
@variable(m, slack4[1:DW, 1:DR, 0:DK] >=0, Int)

# Needed for resources update (remove from r when task starts)
@variable(m, slack5[1:DW, 1:DR, 0:DK] >=0, Int)


# --- COST FUNCTION ---
# --- COST FUNCTION AUXILIARY VARIABLES
# Definition of auxiliary variables for objective function
@variable(m, tau >= 0, start=0, Int);

println("All Variables has been successfully created\n")

# CONSTRAINTS
# --- TASKS ---

# Unique i and k for at
@constraint(m, uniqueness_a_workstation_time[j in 1:DT], sum(sum(at[i,j,k] for k in 1:DK) for i in 1:DW) == 1)

# Duration assignment by workstations for specific task
@constraint(m, duration_assignment[i in 1:DW, j in 1:DT, k in 1:DK], at[i,j,k]*d[i,j,k] == T_ij[i,j]*at[i,j,k]);

# Auxiliary constraint for last time instant (COST FUNCTION)
@constraint(m, last_time_instant[i in 1:DW, j in 1:DT, k in 1:DK], tau >= sum(at[i,j,k]*(k+T_ij[i,j]) for k in 1:DK))

# Starting time s=1 if a=1    (note that if sum=0 s=1 can be)
@constraint(m, starting_assignment[i in 1:DW, j in 1:DT, k in 1:DK], s[i,j,k] >= sum(at[i,j,kk] for kk in 1:k)); 

# Starting time not decreasing
@constraint(m, starting_not_decreasing[i in 1:DW, j in 1:DT, k in 1:DK-1], s[i,j,k] <= s[i,j,k+1]);

# Unique workstation for each s
@constraint(m, unique_workstation_starting_assignment[j in 1:DT, k in 1:DK], sum(s[i,j,k] for i in 1:DW) <= 1);

# Not going back from finishing
@constraint(m, finishing_notgoingback[i in 1:DW, j in 1:DT, k in 1:DK-1], f[i,j,k] <= f[i,j,k+1])

# Tasks deadlines
@constraint(m, deadlines[i in 1:DW, j in 1:DT], sum(d[i,j,k] for k in T_j[j]:DK) == 0)

# Force execution when a=1
@constraint(m, force_execution[i in 1:DW, j in 1:DT, k in 1:DK], at[i,j,k]*e[i,j,k] == at[i,j,k])

# Unique workstation of execution
@constraint(m, unique_execution[j in 1:DT, k in 1:DK], sum(e[i,j,k] for i in 1:DW) <= 1)

# Exectution-finishing states link
@constraint(m, execution_finishing_link[i in 1:DW, j in 1:DT, k in 1:DK], f[i,j,k] + e[i,j,k] == s[i,j,k])

# Hold execution until duration is positive
@constraint(m, execution_lessthan_duration[i in 1:DW, j in 1:DT, k in 1:DK], e[i,j,k] <= d[i,j,k])

# Slack variable constraint
@constraint(m, slack_as_execution[i in 1:DW, j in 1:DT, k in 1:DK], slack[i,j,k] == e[i,j,k])

# Duration update step
@constraint(m, duration_update[i in 1:DW, j in 1:DT, k in 1:DK-1], d[i,j,k+1] == d[i,j,k] - slack[i,j,k])

# Duration not increasing
@constraint(m, duration_not_increasing[i in 1:DW, j in 1:DT, k in 1:DK-1], d[i,j,k] >= d[i,j,k+1]);

# Slack2 variable definition
@constraint(m, slack2_definition[i in 1:DW, j in 1:DT, k in 1:DK], slack2[i,j,k] == e[i,j,k-1]*f[i,j,k])

# Slack2 variable uniqueness
@constraint(m, slack2_proprieties[j in 1:DT], sum(sum(slack2[i,j,k] for i in 1:DW) for k in 0:DK) == 1)

# Time precedence constraints
for j1 in 1:length(T_jj[:,1])
    for j2 in 1:length(T_jj[1,:])
        if T_jj[j1,j2] == 1
            @constraint(m, [i in 1:DW, k in 1:DK], slack2[i,j1,k] => {sum(sum(at[ii,j2,kk] for ii in 1:DW) for kk in k:DK) == 1})
            @constraint(m, [i in 1:DW, k in 1:DK], !slack2[i,j1,k] => {sum(sum(at[ii,j2,kk] for ii in 1:DW) for kk in 0:DK) == 1})
        end
    end
end

# --- RESOURCES ---
# Slack variable definition for available resources update
@constraint(m, slack3_definition[rr in 1:DR, k in 0:DK], slack3[rr,k] == sum(sum(ar[i,j,rr,k]*T_jr[j,rr] for j in 1:DT) for i in 1:DW))

# Slack variable definition for resources updates (from R to r) and (from r to zero)
@constraint(m, slack4_definition[i in 1:DW, rr in 1:DR, k in 0:DK], slack4[i,rr,k] == sum(ar[i,j,rr,k]*T_jr[j,rr] for j in 1:DT))
@constraint(m, slack5_definition[i in 1:DW, rr in 1:DR, k in 1:DK-1], slack5[i,rr,k+1] == sum(e[i,j,k]*f[i,j,k+1]*T_jr[j,rr] for j in 1:DT))

# Task-Resources-Workstation allociation constrainted to be in the same instant of task-workstation allocation, for each resource
@constraint(m, at_ar_link[i in 1:DW, j in 1:DT, r in 1:DR, k in 0:DK], ar[i,j,r,k] == at[i,j,k])

# Occupancy level of workstation definition
@constraint(m, workstation_occupancy[i in 1:DW, k in 1:DK], o[i,k] == sum(e[i,j,k] for j in 1:DT));

# Maximum occupancy level of workstations
@constraint(m, workstation_max_occupancy[i in 1:DW, k in 1:DK], o[i,k] <= W_i[i]);

# Available resources initialization
@constraint(m, init_overall_resources_available[r in 1:DR], R[r,0] == R_r[r])

# Available resources dynamics
@constraint(m, available_resources_dynamics[rr in 1:DR, k in 0:DK-1], R[rr,k+1] == R[rr,k] - slack3[rr,k+1])

# Stored resources definition in k=1
@constraint(m, resources_in_workstation_definition[i in 1:DW, rr in 1:DR], r[i,rr,1] == slack4[i,rr,1]);

# Stored resources dyn
@constraint(m, resources_in_workstation_dyn[i in 1:DW, rr in 1:DR, k in 1:DK-1], r[i,rr,k+1] == r[i,rr,k] + slack4[i,rr,k+1] - slack5[i,rr,k+1]);

println("All constraints defined successfully\n")


# OBJECTIVE FUNCTION AND OPTIMIZATION
# Objective function
obj = tau;

# Definition of objective function
@objective(m, Min, obj)

println("Objective function correctly defined.")

# FUNCTION NEEDED FOR MPC SCHEME
# Function to solve the model
function solve(m)
    optimize!(m)
    
    d_opt = JuMP.value.(d[:,:,:])
    s_opt = JuMP.value.(s[:,:,:])
    e_opt = JuMP.value.(e[:,:,:])
    f_opt = JuMP.value.(f[:,:,:])
    at_opt = JuMP.value.(at[:,:,:])
    slack_opt = JuMP.value.(slack[:,:,:])
    slack2_opt = JuMP.value.(slack2[:,:,:])
    o_opt = JuMP.value.(o[:,:])
    R_opt = JuMP.value.(R[:,:])
    r_opt = JuMP.value.(r[:,:,:])
    ar_opt = JuMP.value.(ar[:,:,:,:])
    slack3_opt = JuMP.value.(slack3[:,:])
    slack4_opt = JuMP.value.(slack4[:,:,:])
    slack5_opt = JuMP.value.(slack5[:,:,:])

    return m, d_opt, s_opt, e_opt, f_opt, at_opt, slack_opt, slack2_opt, o_opt, R_opt, r_opt, ar_opt, slack3_opt, slack4_opt, slack5_opt
end

# Function to update the variables of the model
function update(m, iter, d_opt, s_opt, e_opt, f_opt, at_opt, slack_opt, slack2_opt, o_opt, R_opt, r_opt, ar_opt, slack3_opt, slack4_opt, slack5_opt)
    for k in 0:iter
        
        # FIRST UPDATE LOOP
        for i in 1:DW
            JuMP.fix(o[i,iter], o_opt[i,iter]; force = true) 

            for j in 1:DT
                JuMP.fix(d[i,j,iter], d_opt[i,j,iter]; force = true) 
                JuMP.fix(s[i,j,iter], s_opt[i,j,iter]; force = true)
                JuMP.fix(e[i,j,iter], e_opt[i,j,iter]; force = true)
                JuMP.fix(f[i,j,iter], f_opt[i,j,iter]; force = true)
                JuMP.fix(at[i,j,iter], at_opt[i,j,iter]; force = true)
                JuMP.fix(slack[i,j,iter], slack_opt[i,j,iter]; force = true)
                JuMP.fix(slack2[i,j,iter], slack2_opt[i,j,iter]; force = true)


                for rr in 1:DR
                    JuMP.fix(ar[i,j,rr,iter], ar_opt[i,j,rr,iter]; force = true) 
                end

            end

        end

        # SECOND UPDATE LOOP
        for rr in 1:DR
            JuMP.fix(R[rr,iter], R_opt[rr,iter]; force = true)
            JuMP.fix(slack3[rr,iter], slack3_opt[rr,iter]; force = true)



            for i in 1:DW
                JuMP.fix(r[i,rr,iter], r_opt[i,rr,iter]; force = true)

                JuMP.fix(slack4[i,rr,iter], slack4_opt[i,rr,iter]; force = true)
                JuMP.fix(slack5[i,rr,iter], slack5_opt[i,rr,iter]; force = true)

            end
        end

    end
    return m
end


# Dynamic disturbance delay (to be used in the MPC loop)
function delay(m, i, j, Delta, iter)
    if iter in Delta
        # duration update is not executed
        delete(m, slack_as_execution[i,j,iter])
        JuMP.fix(slack[i,j,iter], 0)
        
        # resources are not consumed if it is not executed
        delete(m,slack5_definition[i,j,iter])
        JuMP.fix(slack5[i,j,iter], 0)
        
        # Auxiliary constraint for last time instant (COST FUNCTION)
        
        # Delete tau constraint if iter == first time in which delay happen (Delta[1])
        if iter == Delta[1]
            for i in 1:DW
                for j in 1:DT
                    for k in 1:DK
                        delete(m, last_time_instant[i,j,k])
                    end
                end
            end
            
            # Add new Tau constraint dependent on (i,j) delay
            @constraint(m, last_time_instant_delay[i in 1:DW, j in 1:DT, k in 1:DK], tau >= sum(at[i,j,k]*(k+T_ij[i,j]) for k in 1:DK) + length(Delta))
        end
    end
    
    
    
    return m
end

println("All the functions needed for the MPC scheme has been defined correctly.")

length(Delta)

# MPC EXECUTION
# Control horizon (Recieding)
DITER = 20;

print("OPTIMAL CONTROL SOLUTION: \n")
# Find optimal solutions for k=1
(m, d_opt, s_opt, e_opt, f_opt, at_opt, slack_opt, slack2_opt, o_opt, R_opt, r_opt, ar_opt, slack3_opt, slack4_opt, slack5_opt) = solve(m)
print("\n\n\n")

print("MODEL PREDICTIVE CONTROL: \n")
for iter in 1:DITER
    println("ITERATION: $iter")
    
    # Fix variables to their optimum value in k=1 , or iteratevely k=iter
    m = update(m, iter, d_opt, s_opt, e_opt, f_opt, at_opt, slack_opt, slack2_opt, o_opt, R_opt, r_opt, ar_opt, slack3_opt, slack4_opt, slack5_opt)
    
    
    # Update variables according to Delay
    m = delay(m, i, j, Delta, iter)

    # Solve the model with the fixed variables up to k=iter
    (m, d_opt, s_opt, e_opt, f_opt, at_opt, slack_opt, slack2_opt, o_opt, R_opt, r_opt, ar_opt, slack3_opt, slack4_opt, slack5_opt) = solve(m)
    
    
    println("Best optimal time in iteration $iter:")
    println(JuMP.value.(tau))
    println("Best founded objective value in iteration $iter:")
    println(objective_value(m))
    println("\n")

    println("END of iteration $iter \n\n\n\n")
    
end

println("END of the MPC")

## PLOTS
# Time interval definition
K = [0:DK];

# DURATIONS
# Create data structure for the output of the model
D = [];
for i in 1:DW
    di = []
    
    for j in 1:DT
        dj = []
        
        for k in 0:DK
            push!(dj, JuMP.value.(d[i,j,k])) 
        end
        push!(di, dj)
        
    end
    push!(D, di)
    
end

# Create plots
for i in 1:DW
    KK = []
    durations = []
    
    labdj = []
    
    for j in 1:DT
        x = convert(Array{Float64,1}, D[i][j])
        push!(durations, x)
        push!(KK, K)
        
        push!(labdj, "d[$j]")
        
    end
    display(plot(KK, durations, title="Durations in Workstation: $i", lw=1, labels=permutedims(labdj), linetype="steppost", size = (400, 300)))
    
end


# STATES (EXECTUING-FINISHING-ASSIGNMENT(Control))
# Create data structure for the output of the model
E = [];
F = [];
AT = [];

for i in 1:DW
    sei = []
    sfi = []
    sati = []
    
    labei = []
    labfi = []
    labati = []
    
    for j in 1:DT
        sej = []
        sfj = []
        satj = []
        
        for k in 0:DK
            push!(sej, JuMP.value.(e[i,j,k]))
            push!(sfj, JuMP.value.(f[i,j,k])) 
            push!(satj, JuMP.value.(at[i,j,k])) 

        end
        push!(sei, sej)
        push!(sfi, sfj)
        push!(sati, satj)

    end
    push!(E, sei)
    push!(F, sfi)
    push!(AT, sati)
    
end

# Create plots
for i in 1:DW
    KK = []
    execution = []
    finishing = []
    assignment = []
    
    labej = []
    labfj = []
    labatj = []
        

    for j in 1:DT        
        xe = convert(Array{Float64,1}, E[i][j])
        xf = convert(Array{Float64,1}, F[i][j])
        xat = convert(Array{Float64,1}, AT[i][j])
        push!(execution, xe)
        push!(finishing, xf)
        push!(assignment, xat)
        push!(KK, K)
        
        push!(labej, "e[$(i),$(j)]")
        push!(labfj, "f[$(i),$(j)]")
        push!(labatj, "at[$(i),$(j)]")

    end
    
    display(plot(KK, [execution, finishing], title="States of Workstation: $i", labels = [permutedims(labej) permutedims(labfj)] , lw=3, size = (400, 300)))
    display(plot(KK, assignment, title="Assignment to Workstation: $i", labels = permutedims(labatj), lw=2, size = (400, 300)))
    
end


# OCCUPANCY
# Create data structure for the output of the model
O = [];
for i in 1:DW
    oi = []
            
    for k in 0:DK
        push!(oi, JuMP.value.(o[i,k])) 
    end
        
    push!(O, oi)
    
end

# Create plots
K = [0:DK];

for i in 1:DW
    KK = []
    occupancy = []
    
    
    x = convert(Array{Float64,1}, O[i])
    push!(occupancy, x)
    push!(KK, K)
    
    display(plot(KK, occupancy, title="Occupancy of Workstation: $i", lw=2, labels="o[$i]" , linetype="steppost", size = (400, 300)))
    
end


# RESOURCES
# Create data structure for the output of the model
RR = [];
AR = [];
rR = [];

for rr in 1:DR
    #Scaling_Factor = JuMP.value.(R[r,0])*0.1
    Scaling_Factor = 10

    Rr = []
    arr = []
    rrr = []
    
    for k in 0:DK
        yy = 0
        y = 0
        
        for i in 1:DW
            y += JuMP.value.(r[i,rr,k])
            
            for j in 1:DT
                yy += JuMP.value.(ar[i,j,rr,k])            
            end

        end
    
    push!(Rr, JuMP.value.(R[rr,k]))
    push!(arr, yy*Scaling_Factor)
    push!(rrr, y)
        
    end
    
    push!(RR, Rr)
    push!(AR, arr)
    push!(rR, rrr)
end


# Create plots
K = [0:DK];

for rr in 1:DR
    KK = []
    allresources = []
    assignmentres = []
    storedresources = []

    labRr = []
    labarr = []
    labrrr = []
    
    x1 = convert(Array{Float64,1}, RR[rr])
    x2 = convert(Array{Float64,1}, rR[rr])
    x3 = convert(Array{Float64,1}, AR[rr])

    push!(allresources, x1)
    push!(storedresources, x2)
    push!(assignmentres, x3)    
    push!(KK, K)
    
    push!(labRr, "R[$rr]")
    push!(labarr, "ar[$rr] summed on all i and all j (scaled)")
    push!(labrrr, "r[$rr] summed on all i")

    display(plot(KK, [allresources, storedresources, assignmentres] , title="Resources Dynamics of Resource Type: $rr", labels = [permutedims(labRr) permutedims(labrrr) permutedims(labarr)], lw=2, linetype=["steppost" "steppost" "line"], size = (400, 400)))
    
end




