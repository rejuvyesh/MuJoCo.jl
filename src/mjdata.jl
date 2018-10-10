

@enum mjtWarning begin             # warning types
   WARN_INERTIA      = 0         # (near) singular inertia matrix
   WARN_CONTACTFULL              # too many contacts in contact list
   WARN_CNSTRFULL                # too many constraints
   WARN_VGEOMFULL                # too many visual geoms
   WARN_BADQPOS                  # bad number in qpos
   WARN_BADQVEL                  # bad number in qvel
   WARN_BADQACC                  # bad number in qacc
   WARN_BADCTRL                  # bad number in ctrl

   NWARNING                      # number of warnings
end


@enum mjtTimer begin
   # main api
   TIMER_STEP        = 0         # step
   TIMER_FORWARD                 # forward
   TIMER_INVERSE                 # inverse

   # breakdown of step/forward
   TIMER_POSITION                # fwdPosition
   TIMER_VELOCITY                # fwdVelocity
   TIMER_ACTUATION               # fwdActuation
   TIMER_ACCELERATION            # fwdAcceleration
   TIMER_CONSTRAINT              # fwdConstraint

   # breakdown of fwdPosition
   TIMER_POS_KINEMATICS          # kinematics, com, tendon, transmission
   TIMER_POS_INERTIA             # inertia computations
   TIMER_POS_COLLISION           # collision detection
   TIMER_POS_MAKE                # make constraints
   TIMER_POS_PROJECT             # project constraints

   NTIMER                        # number of timers
end


#------------------------------ mjContact ----------------------------------------------
struct mjContact
   dist::mjtNum
   pos::SVector{3, mjtNum}
   frame::SVector{9, mjtNum}
   includemargin::mjtNum
   friction::SVector{5, mjtNum}
   solref::SVector{NREF, mjtNum}
   solimp::SVector{NIMP, mjtNum}
   mu::mjtNum
   H::SVector{36, mjtNum}
   dim::Cint
   geom1::Cint
   geom2::Cint
   exclude::Cint
   efc_address::Cint
end

#------------------------------ diagnostics --------------------------------------------

struct mjWarningStat
   lastinfo::Cint
   number::Cint
end

struct mjTimerStat
   duration::mjtNum
   number::Cint
end

struct mjSolverStat
   improvement::mjtNum
   gradient::mjtNum
   lineslope::mjtNum
   nactive::Cint
   nchange::Cint
   neval::Cint
   nupdate::Cint
end

#---------------------------------- mjData ---------------------------------------------

mutable struct mjData
   nstack::Cint
   nbuffer::Cint

   pstack::Cint

   maxuse_stack::Cint
   maxuse_con::Cint
   maxuse_efc::Cint

   warning::SVector{Int(NWARNING), mjWarningStat}
   timer::SVector{Int(NTIMER), mjTimerStat}
   solver::SVector{NSOLVER, mjSolverStat}
   solver_iter::Cint
   solver_nnz::Cint
   solver_fwdinv::SVector{2, mjtNum}

   ne::Cint
   nf::Cint
   nefc::Cint
   ncon::Cint
   time::mjtNum
   energy::SVector{2, mjtNum}

   buffer::Ptr{Cvoid}
   stack::Ptr{mjtNum}

   qpos::Ptr{mjtNum}
   qvel::Ptr{mjtNum}
   act::Ptr{mjtNum}
   qacc_warmstart::Ptr{mjtNum}

   ctrl::Ptr{mjtNum}
   qfrc_applied::Ptr{mjtNum}
   xfrc_applied::Ptr{mjtNum}

   qacc::Ptr{mjtNum}
   act_dot::Ptr{mjtNum}

   mocap_pos::Ptr{mjtNum}
   mocap_quat::Ptr{mjtNum}

   userdata::Ptr{mjtNum}

   sensordata::Ptr{mjtNum}

   xpos::Ptr{mjtNum}
   xquat::Ptr{mjtNum}
   xmat::Ptr{mjtNum}
   xipos::Ptr{mjtNum}
   ximat::Ptr{mjtNum}
   xanchor::Ptr{mjtNum}
   xaxis::Ptr{mjtNum}
   geom_xpos::Ptr{mjtNum}
   geom_xmat::Ptr{mjtNum}
   site_xpos::Ptr{mjtNum}
   site_xmat::Ptr{mjtNum}
   cam_xpos::Ptr{mjtNum}
   cam_xmat::Ptr{mjtNum}
   light_xpos::Ptr{mjtNum}
   light_xdir::Ptr{mjtNum}

   subtree_com::Ptr{mjtNum}
   cdof::Ptr{mjtNum}
   cinert::Ptr{mjtNum}

   ten_wrapadr::Ptr{Cint}
   ten_wrapnum::Ptr{Cint}
   ten_J_rownnz::Ptr{Cint}
   ten_J_rowadr::Ptr{Cint}
   ten_J_colind::Ptr{Cint}
   ten_length::Ptr{mjtNum}
   ten_J::Ptr{mjtNum}
   wrap_obj::Ptr{Cint}
   wrap_xpos::Ptr{mjtNum}

   actuator_length::Ptr{mjtNum}
   actuator_moment::Ptr{mjtNum}

   crb::Ptr{mjtNum}
   qM::Ptr{mjtNum}
   qLD::Ptr{mjtNum}
   qLDiagInv::Ptr{mjtNum}
   qLDiagSqrtInv::Ptr{mjtNum}

   contact::Ptr{mjContact}

   efc_type::Ptr{Cint}
   efc_id::Ptr{Cint}
   efc_J_rownnz::Ptr{Cint}
   efc_J_rowadr::Ptr{Cint}
   efc_J_rowsuper::Ptr{Cint}
   efc_J_colind::Ptr{Cint}
   efc_JT_rownnz::Ptr{Cint}
   efc_JT_rowadr::Ptr{Cint}
   efc_JT_rowsuper::Ptr{Cint}
   efc_JT_colind::Ptr{Cint}
   efc_J::Ptr{mjtNum}
   efc_JT::Ptr{mjtNum}
   efc_pos::Ptr{mjtNum}
   efc_margin::Ptr{mjtNum}
   efc_frictionloss::Ptr{mjtNum}
   efc_diagApprox::Ptr{mjtNum}
   efc_KBIP::Ptr{mjtNum}
   efc_D::Ptr{mjtNum}
   efc_R::Ptr{mjtNum}

   efc_AR_rownnz::Ptr{Cint}
   efc_AR_rowadr::Ptr{Cint}
   efc_AR_colind::Ptr{Cint}
   efc_AR::Ptr{mjtNum}

   ten_velocity::Ptr{mjtNum}
   actuator_velocity::Ptr{mjtNum}
   cvel::Ptr{mjtNum}
   cdof_dot::Ptr{mjtNum}
   qfrc_bias::Ptr{mjtNum}
   qfrc_passive::Ptr{mjtNum}
   efc_vel::Ptr{mjtNum}
   efc_aref::Ptr{mjtNum}
   subtree_linvel::Ptr{mjtNum}
   subtree_angmom::Ptr{mjtNum}
   actuator_force::Ptr{mjtNum}
   qfrc_actuator::Ptr{mjtNum}
   qfrc_unc::Ptr{mjtNum}
   qacc_unc::Ptr{mjtNum}

   efc_b::Ptr{mjtNum}
   efc_force::Ptr{mjtNum}
   efc_state::Ptr{Cint}
   qfrc_constraint::Ptr{mjtNum}

   qfrc_inverse::Ptr{mjtNum}

   cacc::Ptr{mjtNum}
   cfrc_int::Ptr{mjtNum}
   cfrc_ext::Ptr{mjtNum}
end

# Callback function types TODO

const mjfGeneric = Ptr{Cvoid}
const mjfConFilt = Ptr{Cvoid}
const mjfSensor = Ptr{Cvoid}
const mjfTime = Ptr{Cvoid}
const mjfAct = Ptr{Cvoid}
const mjfCollision = Ptr{Cvoid}


