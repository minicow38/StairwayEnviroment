using UnityEngine;
using System.Collections;
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider))]
public sealed class SlopeStick3DCompact : MonoBehaviour
{
    const float Epsilon = 0.000001f;
    const float MinSlopeAngle = 2f, MaxSlopeAngle = 75f;
    const float GroundAcceleration = 35f, AirAcceleration = 6f, MaxDeceleration = 80f;
    const float ResponseInverse = 8.333333f, AccelerationJerk = 600f;
    const float SurfaceConfirmDistance = 0.40f, MaxSurfaceScore = 2.25f;
    const float EntryEnd = 0.25f, BridgeMaxGap = 0.18f, BridgeRecoveryLimit = 180f;
    const float SphereRadius = 0.50f, ProbeRadius = 0.475f, ProbeDistance = 0.50f;
    const float SupportGraceDistance = 0.95f, ForwardProbeStep = 0.25f;
    const int ForwardProbeCount = 32;
    const float ForwardProbeHeight = 2.5f, ForwardProbeDown = 7f, ForwardRetentionDistance = 0.70f;
    const float ConnectedProbeOffset = 0.50f, ConnectedMaxGap = 2f, CurvatureDistance = 0.75f;
    const float MinCurvature = 0.0005f, MaxCurvature = 1f;
    const float TargetMinDistance = 0.27f, TargetAccelerationLimit = 120f;
    const float PostTargetBlendWidth = 0.05f, PostTargetGravityRatio = 0.50f;
    const float CriticalRatio = 0.98f, InverseCriticalRatio = 1f / CriticalRatio;
    const float FlatStick = 24.6f, MaxStick = 1000f, StickPredictionSeconds = 0.06f;
    const float OutwardResponseInverse = 12.5f, StickSafety = 1.08f;
    const float StickRiseJerk = 5000f, StickFallJerk = 1200f;
    const float ReleaseHold = 0.02f, MinimumReleaseWidth = 0.08f, NaturalReleaseEnd = 0.90f;
    public GameObject sub;
    struct Observation
    {
        public Collider collider;
        public Vector3 point, normal;
        public bool Valid => collider != null;
    }

    struct SlopeFrame
    {
        public Collider collider;
        public Vector3 normal, supportNormal, supportPoint, axis, entry, exit;
        public float length, entryCurvature, exitCurvature, curvature;
        public bool Valid => collider != null && length > Epsilon;
        public bool HasSupport => supportNormal.sqrMagnitude > Epsilon;
    }

    struct Surface
    {
        public Vector3 tangent, side, normal;
        public float tangentSpeed, lateralSpeed, outwardSpeed, gravityAlong, gravitySupport;
        public bool Valid => tangent.sqrMagnitude > Epsilon;
    }

    struct TrackingState
    {
        public SlopeFrame active, forward;
        public Collider candidateCollider;
        public float stableDistance, supportMissDistance, candidateDistance, forwardMissDistance;
    }

    struct ControlState
    {
        public float groundAcceleration, targetAcceleration, stick, capturedTargetSpeed;
        public bool released;
    }

    [Header("Travel")] [Min(0f)] [SerializeField] float maxGroundSpeed = 16f;
    [Range(0f, 100f)] [SerializeField] float targetSlopeProgressPercent = 60f;

    [Header("Environment")] [SerializeField] Vector3 travelDirection = Vector3.forward;
    [SerializeField] LayerMask groundMask = ~0;
    Rigidbody rb;
    Vector3 direction;
    TrackingState tracking;
    ControlState control;
    float TargetProgress => Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);
    float ReleaseStart => Mathf.Clamp01(TargetProgress + ReleaseHold);
    float ReleaseEnd => Mathf.Clamp01(Mathf.Max(NaturalReleaseEnd, ReleaseStart + MinimumReleaseWidth));
    float ObserverReferenceSpeed => Mathf.Max(0.25f, maxGroundSpeed * 0.10f);
    float BridgeMaxOutwardSpeed => Mathf.Max(0.25f, maxGroundSpeed * 0.078125f);
    float ReverseTolerance => Mathf.Max(0.10f, maxGroundSpeed / 64f);
    bool IsEntry => tracking.active.Valid && Progress(tracking.active) < EntryEnd;

    // Rank 01: BuildSurface
    Surface BuildSurface(Vector3 normalInput, Vector3 preferredTangent, Vector3 alignmentAxis)
    {
        Vector3 normal = normalInput.sqrMagnitude > Epsilon ? normalInput.normalized : Vector3.up;
        Vector3 tangent = Vector3.ProjectOnPlane(preferredTangent, normal);
        if (tangent.sqrMagnitude <= Epsilon)
        {
            tangent = Vector3.ProjectOnPlane(direction, normal);
        }
        if (tangent.sqrMagnitude <= Epsilon) tangent = Vector3.Cross(normal, Vector3.right);
        if (tangent.sqrMagnitude <= Epsilon) tangent = Vector3.Cross(normal, Vector3.forward);
        if (tangent.sqrMagnitude <= Epsilon) return default;
        tangent.Normalize();
        Vector3 side = Vector3.Cross(normal, tangent);
        if (side.sqrMagnitude <= Epsilon) return default;
        side.Normalize();
        tangent = Vector3.Cross(side, normal).normalized;
        if (alignmentAxis.sqrMagnitude > Epsilon && Vector3.Dot(tangent, alignmentAxis) < 0f)
        {
            tangent = -tangent;
            side = -side;
        }
        Vector3 velocity = rb.velocity;
        return new Surface {
            tangent = tangent, side = side, normal = normal, tangentSpeed = Vector3.Dot(velocity, tangent), lateralSpeed = Vector3.Dot(velocity, side), outwardSpeed = Mathf.Max(0f, Vector3.Dot(velocity, normal)), gravityAlong = Vector3.Dot(Physics.gravity, tangent), gravitySupport = Mathf.Max(0f, Vector3.Dot(Physics.gravity, -normal))
        };
    }

    // Rank 02: ApplySurfaceAcceleration
    void ApplySurfaceAcceleration(Surface surface, float tangentialAcceleration, float inwardAcceleration)
    {
        Vector3 acceleration = surface.tangent * tangentialAcceleration - surface.side * (surface.lateralSpeed * ResponseInverse) - surface.normal * inwardAcceleration;
        rb.AddForce(acceleration, ForceMode.Acceleration);
    }

    // Rank 03: ObserveGround
    Observation ObserveGround()
    {
        Vector3 origin = rb.worldCenterOfMass + Vector3.up * 0.05f;
        if (!Physics.SphereCast(origin, ProbeRadius, Vector3.down, out RaycastHit hit, ProbeDistance, groundMask, QueryTriggerInteraction.Ignore) || Vector3.Angle(hit.normal, Vector3.up) > MaxSlopeAngle)
        {
            return default;
        }
        return new Observation {
            collider = hit.collider, point = hit.point, normal = hit.normal.normalized
        };
    }

    // Rank 04: FixedUpdate
    void FixedUpdate()
    {
        bool wasEntry = IsEntry;
        Observation observation = ObserveGround();
        bool grounded = observation.Valid;
        bool bridged = false;
        bool entryReadAsFlat = grounded && Vector3.Angle(observation.normal, Vector3.up) < MinSlopeAngle && wasEntry;
        if ((!grounded || entryReadAsFlat) && tracking.supportMissDistance <= Epsilon)
        {
            Observation bridge = BuildEntryBridge(wasEntry);
            if (bridge.Valid)
            {
                observation = bridge;
                grounded = bridged = true;
            }
        }
        if (!grounded)
        {
            tracking.supportMissDistance += ObserverDistanceStep();
            if (HoldSlopeAfterSupportLoss()) return;
            SolveAir();
            ClearGroundState();
            return;
        }
        tracking.supportMissDistance = 0f;
        float angle = Vector3.Angle(observation.normal, Vector3.up);
        if (angle >= MinSlopeAngle && angle <= MaxSlopeAngle)
        {
            Vector3 measuredNormal = UpdateActiveFrame(observation);
            SolveSlope(IsEntry ? tracking.active.normal : measuredNormal, bridged);
            return;
        }
        bool preserveEntry = wasEntry && IsEntry;
        if (!preserveEntry)
        {
            tracking.stableDistance = 0f;
            ClearCandidate();
            ResetSlopeSession(false);
            if (tracking.active.Valid && tracking.active.collider != observation.collider) tracking.active = default;
        }
        UpdateForwardSlope(observation.normal);
        SolveFlat(observation.normal, !preserveEntry);
    }

    // Rank 05: SolveFlat
    void SolveFlat(Vector3 flatNormal, bool allowForwardControl)
    {
        Surface surface = BuildSurface(flatNormal, direction, Vector3.zero);
        if (!surface.Valid) return;
        float acceleration = Mathf.Clamp((maxGroundSpeed - surface.tangentSpeed) * ResponseInverse, -MaxDeceleration, GroundAcceleration);
        float preload = 0f;
        if (allowForwardControl && tracking.forward.Valid)
        {
            float distance = Vector3.Dot(tracking.forward.entry - rb.position, surface.tangent);
            if (distance > 0f)
            {
                float permitted = Mathf.Sqrt(Mathf.Max(0f, Mathf.Pow(SafeSpeed(tracking.forward), 2f) + 2f * MaxDeceleration * distance));
                if (surface.tangentSpeed > permitted)
                {
                    acceleration = Mathf.Min(acceleration, RequiredAcceleration(Mathf.Max(0f, surface.tangentSpeed), permitted, Mathf.Max(TargetMinDistance, distance)));
                }
                preload = EntryPreload(tracking.forward, distance);
            }
        }
        control.groundAcceleration = Move(control.groundAcceleration, acceleration, AccelerationJerk, AccelerationJerk);
        ApplySurfaceAcceleration(surface, control.groundAcceleration, FlatStick);
        if (preload > 0f && tracking.forward.Valid)
        {
            rb.AddForce(-tracking.forward.normal.normalized * preload, ForceMode.Acceleration);
        }
    }

    // Rank 06: RequiredAcceleration
    static float RequiredAcceleration(float currentSpeed, float targetSpeed, float distance) => (targetSpeed * targetSpeed - currentSpeed * currentSpeed) / (2f * Mathf.Max(0.0001f, distance));

    // Rank 07: Progress
    float Progress(SlopeFrame frame) => frame.Valid ? Mathf.Clamp01(Vector3.Dot(rb.position - frame.entry, frame.axis) / frame.length) : 0f;

    // Rank 08: GroundDrive
    float GroundDrive(float speed)
    {
        float target = Mathf.Clamp((maxGroundSpeed - speed) * ResponseInverse, -MaxDeceleration, GroundAcceleration);
        control.groundAcceleration = Move(control.groundAcceleration, target, AccelerationJerk, AccelerationJerk);
        return control.groundAcceleration;
    }

    // Rank 09: SolveSlope
    void SolveSlope(Vector3 controlNormal, bool bridged)
    {
        if (!tracking.active.Valid)
        {
            SolveAir();
            return;
        }
        Surface surface = BuildSurface(controlNormal, tracking.active.axis, tracking.active.axis);
        if (!surface.Valid) return;
        float progress = Progress(tracking.active);
        EnsureTargetPlan();
        float baseAcceleration = GroundDrive(surface.tangentSpeed);
        float targetAcceleration = TargetDrive(progress, surface.tangentSpeed, surface.gravityAlong);
        float acceleration = control.capturedTargetSpeed >= 0f && progress < TargetProgress ? targetAcceleration : baseAcceleration;
        if (progress >= TargetProgress)
        {
            acceleration -= Mathf.Max(0f, surface.gravityAlong) * PostTargetGravityRatio * SmoothRange01(progress, TargetProgress, Mathf.Clamp01(TargetProgress + PostTargetBlendWidth));
            acceleration = Mathf.Min(0f, acceleration);
        }
        acceleration = Mathf.Max(-MaxDeceleration, acceleration);
        if (bridged && IsEntry) acceleration = Mathf.Min(0f, acceleration);
        float release = 1f - SmoothRange01(progress, ReleaseStart, ReleaseEnd);
        control.released = progress >= ReleaseEnd;
        acceleration *= release;
        float stick = AdaptiveStick(surface.tangentSpeed, acceleration, EvaluateCurvature(tracking.active, progress), surface.gravitySupport, surface.outwardSpeed) * release;
        if (bridged)
        {
            stick = Mathf.Max(stick, BridgeRecoveryAcceleration());
        }
        ApplySurfaceAcceleration(surface, acceleration, stick);
    }

    // Rank 10: BuildSlopeFrame
    SlopeFrame BuildSlopeFrame(Collider collider, Vector3 referencePoint, Vector3 referenceNormal, Vector3 desiredMove)
    {
        if (collider == null) return default;
        Vector3 axis = Vector3.ProjectOnPlane(desiredMove, referenceNormal);
        if (axis.sqrMagnitude <= Epsilon && tracking.active.Valid) axis = Vector3.ProjectOnPlane(tracking.active.axis, referenceNormal);
        if (axis.sqrMagnitude <= Epsilon) axis = Vector3.ProjectOnPlane(direction, referenceNormal);
        if (axis.sqrMagnitude <= Epsilon) return default;
        axis.Normalize();
        if (Vector3.Dot(axis, desiredMove) < 0f) axis = -axis;
        if (!TryGetProjectionRange(collider, axis, out float min, out float max)) return default;
        float referenceCoordinate = Vector3.Dot(referencePoint, axis);
        SlopeFrame frame = new SlopeFrame {
            collider = collider, normal = referenceNormal.normalized, supportNormal = referenceNormal.normalized, supportPoint = referencePoint, axis = axis, entry = referencePoint + axis * (min - referenceCoordinate), exit = referencePoint + axis * (max - referenceCoordinate), length = Mathf.Max(0.01f, max - min)
        };
        PopulateCurvature(ref frame);
        return frame;
    }

    // Rank 11: Move
    static float Move(float current, float target, float riseJerk, float fallJerk)
    {
        float jerk = target >= current ? riseJerk : fallJerk;
        return Mathf.MoveTowards(current, target, jerk * Time.fixedDeltaTime);
    }

    // Rank 12: TargetDrive
    float TargetDrive(float progress, float speed, float gravityAlong)
    {
        float target = 0f;
        if (control.capturedTargetSpeed >= 0f && progress < TargetProgress)
        {
            float remaining = Mathf.Max(TargetMinDistance, (TargetProgress - progress) * tracking.active.length);
            target = Mathf.Clamp(RequiredAcceleration(speed, control.capturedTargetSpeed, remaining) - gravityAlong, -TargetAccelerationLimit, TargetAccelerationLimit);
        }
        control.targetAcceleration = Move(control.targetAcceleration, target, AccelerationJerk, AccelerationJerk);
        return control.targetAcceleration;
    }

    // Rank 13: TryGetProjectionRange
    bool TryGetProjectionRange(Collider collider, Vector3 axis, out float min, out float max)
    {
        min = float.PositiveInfinity;
        max = float.NegativeInfinity;
        if (collider is BoxCollider box)
        {
            Vector3 half = box.size * 0.5f;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
            {
                Vector3 local = box.center + Vector3.Scale(half, new Vector3(x, y, z));
                IncludeProjection(box.transform.TransformPoint(local), axis, ref min, ref max);
            }
        }
        else
        {
            Bounds bounds = collider.bounds;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
            {
                IncludeProjection(new Vector3(x < 0 ? bounds.min.x : bounds.max.x, y < 0 ? bounds.min.y : bounds.max.y, z < 0 ? bounds.min.z : bounds.max.z), axis, ref min, ref max);
            }
        }
        return min < max;
    }

    // Rank 14: EnsureTargetPlan
    void EnsureTargetPlan()
    {
        if (control.capturedTargetSpeed >= 0f) return;
        float step = SurfaceTravelStep(tracking.active);
        float requiredDistance = Mathf.Max(SurfaceConfirmDistance, step * 2f);
        if (tracking.stableDistance < requiredDistance) return;
        control.capturedTargetSpeed = Mathf.Min(maxGroundSpeed, SafeSpeed(tracking.active));
    }

    // Rank 15: UpdateActiveFrame
    Vector3 UpdateActiveFrame(Observation observation)
    {
        if (tracking.active.Valid && tracking.active.collider == observation.collider)
        {
            ClearCandidate();
            tracking.active.normal = BlendUnitVectors(tracking.active.normal, observation.normal, 0.20f);
            tracking.active.supportNormal = observation.normal;
            tracking.active.supportPoint = observation.point;
            tracking.stableDistance += SurfaceTravelStep(tracking.active);
            return observation.normal;
        }
        bool promoted = tracking.forward.Valid && tracking.forward.collider == observation.collider;
        SlopeFrame next = promoted ? tracking.forward : BuildSlopeFrame(observation.collider, observation.point, observation.normal, direction);
        if (!next.Valid) return RetainedNormal();
        if (promoted && Vector3.Dot(next.normal, observation.normal) < 0f) next.normal = -next.normal;
        if (tracking.active.Valid && !ConfirmSurfaceSwitch(observation, next)) return RetainedNormal();
        next.supportNormal = observation.normal;
        next.supportPoint = observation.point;
        tracking.active = next;
        tracking.stableDistance = SurfaceTravelStep(next);
        ResetSlopeSession(false);
        ClearCandidate();
        if (promoted)
        {
            ApplyEntryVelocityTransport(next);
            tracking.forward = default;
            tracking.forwardMissDistance = 0f;
        }
        return observation.normal;
    }

    // Rank 16: AdaptiveStick
    float AdaptiveStick(float speed, float tangentialAcceleration, float curvature, float gravitySupport, float outwardSpeed)
    {
        float target = FlatStick;
        if (curvature > MinCurvature)
        {
            float forecast = Mathf.Max(0f, speed + Mathf.Max(0f, tangentialAcceleration) * StickPredictionSeconds);
            float required = forecast * forecast * curvature * InverseCriticalRatio - gravitySupport;
            target = Mathf.Min(MaxStick, Mathf.Max(0f, required) * StickSafety + outwardSpeed * OutwardResponseInverse);
        }
        control.stick = Move(control.stick, target, StickRiseJerk, StickFallJerk);
        return control.stick;
    }

    // Rank 17: SafeSpeed
    float SafeSpeed(SlopeFrame frame)
    {
        float curvature = Mathf.Max(MinCurvature, frame.curvature);
        float gravitySupport = Mathf.Max(0f, Vector3.Dot(Physics.gravity, -frame.normal.normalized));
        return Mathf.Sqrt(Mathf.Max(0f, CriticalRatio * (gravitySupport + MaxStick) / curvature));
    }

    // Rank 18: NormalizeFlat
    static Vector3 NormalizeFlat(Vector3 value, Vector3 fallback)
    {
        Vector3 flat = Vector3.ProjectOnPlane(value, Vector3.up);
        if (flat.sqrMagnitude <= Epsilon)
        {
            flat = Vector3.ProjectOnPlane(fallback, Vector3.up);
        }
        if (flat.sqrMagnitude <= Epsilon) flat = Vector3.forward;
        return flat.normalized;
    }

    // Rank 19: SmoothRange01
    static float SmoothRange01(float value, float start, float end)
    {
        if (end <= start + Epsilon) return value >= end ? 1f : 0f;
        return SmootherStep01(Mathf.InverseLerp(start, end, value));
    }

    // Rank 20: SmootherStep01
    static float SmootherStep01(float value)
    {
        float t = Mathf.Clamp01(value);
        return t * t * t * (t * (t * 6f - 15f) + 10f);
    }

    // Rank 21: ConfirmSurfaceSwitch
    bool ConfirmSurfaceSwitch(Observation observation, SlopeFrame next)
    {
        if (SurfaceSwitchScore(observation, next) > MaxSurfaceScore)
        {
            ClearCandidate();
            return false;
        }
        if (tracking.candidateCollider != next.collider)
        {
            tracking.candidateCollider = next.collider;
            tracking.candidateDistance = 0f;
        }
        float step = ObserverDistanceStep();
        tracking.candidateDistance += step;
        if (tracking.candidateDistance < Mathf.Max(SurfaceConfirmDistance, step * 2f)) return false;
        ClearCandidate();
        return true;
    }

    // Rank 22: SurfaceSwitchScore
    float SurfaceSwitchScore(Observation observation, SlopeFrame next)
    {
        float forwardGap = Vector3.Distance(tracking.active.exit, next.entry);
        float backwardGap = Vector3.Distance(tracking.active.entry, next.exit);
        if (IsEntry && Progress(next) > 0.55f && backwardGap + SphereRadius * 0.5f < forwardGap)
        {
            return float.PositiveInfinity;
        }
        float normalCost = (1f - Mathf.Clamp01(Vector3.Dot(tracking.active.normal.normalized, next.normal.normalized))) * 2f;
        float axisCost = (1f - Mathf.Clamp01(Vector3.Dot(tracking.active.axis.normalized, next.axis.normalized))) * 1.5f;
        float supportGap = Mathf.Abs(Vector3.Dot(rb.worldCenterOfMass - observation.point, observation.normal.normalized) - SphereRadius);
        float gapCost = Mathf.Min(2f, supportGap / SphereRadius) * 0.5f;
        float topologyCost = Mathf.Min(1f, forwardGap / (SphereRadius * 8f));
        float backwardPenalty = backwardGap + SphereRadius * 0.5f < forwardGap ? 2f : 0f;
        Vector3 candidateVelocity = Vector3.ProjectOnPlane(rb.velocity, next.normal);
        float reversePenalty = Vector3.Dot(candidateVelocity, next.axis) < -ReverseTolerance ? 3f : 0f;
        return normalCost + axisCost + gapCost + topologyCost + backwardPenalty + reversePenalty;
    }

    // Rank 23: UpdateForwardSlope
    void UpdateForwardSlope(Vector3 flatNormal)
    {
        Vector3 travel = Vector3.ProjectOnPlane(direction, flatNormal);
        if (travel.sqrMagnitude <= Epsilon) return;
        travel.Normalize();
        for (int i = 1; i <= ForwardProbeCount; i++)
        {
            Vector3 sample = rb.position + travel * (ForwardProbeStep * i);
            Vector3 origin = sample + Vector3.up * ForwardProbeHeight;
            if (!Physics.Raycast(origin, Vector3.down, out RaycastHit hit, ForwardProbeDown, groundMask, QueryTriggerInteraction.Ignore))
            {
                continue;
            }
            float angle = Vector3.Angle(hit.normal, Vector3.up);
            if (angle < MinSlopeAngle || angle > MaxSlopeAngle) continue;
            SlopeFrame detected = BuildSlopeFrame(hit.collider, hit.point, hit.normal, travel);
            if (!detected.Valid) continue;
            tracking.forward = detected;
            tracking.forwardMissDistance = 0f;
            return;
        }
        tracking.forwardMissDistance += ObserverDistanceStep();
        if (tracking.forwardMissDistance <= ForwardRetentionDistance) return;
        tracking.forward = default;
        tracking.forwardMissDistance = 0f;
    }

    // Rank 24: BuildEntryBridge
    Observation BuildEntryBridge(bool wasEntry)
    {
        SlopeFrame frame = tracking.active;
        if (!wasEntry || !frame.Valid || !frame.HasSupport || Progress(frame) > EntryEnd) return default;
        Vector3 normal = frame.supportNormal;
        float planeDistance = Vector3.Dot(rb.worldCenterOfMass - frame.supportPoint, normal);
        float gap = Mathf.Abs(planeDistance - SphereRadius);
        float outward = Mathf.Max(0f, Vector3.Dot(rb.velocity, normal));
        if (gap > BridgeMaxGap || outward > BridgeMaxOutwardSpeed) return default;
        return new Observation {
            collider = frame.collider, point = rb.worldCenterOfMass - normal * SphereRadius, normal = normal
        };
    }

    // Rank 25: HoldSlopeAfterSupportLoss
    bool HoldSlopeAfterSupportLoss()
    {
        SlopeFrame frame = tracking.active;
        if (!frame.Valid || !frame.HasSupport || control.released || tracking.supportMissDistance > SupportGraceDistance)
        {
            return false;
        }
        Vector3 normal = frame.supportNormal.normalized;
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(rb.velocity, normal);
        float forward = Vector3.Dot(tangentVelocity, frame.axis);
        float outward = Mathf.Max(0f, Vector3.Dot(rb.velocity, normal));
        if (forward < -ReverseTolerance || outward > BridgeMaxOutwardSpeed) return false;
        SolveObserverGrace(normal);
        return true;
    }

    // Rank 26: SolveObserverGrace
    void SolveObserverGrace(Vector3 normal)
    {
        Surface surface = BuildSurface(normal, tracking.active.axis, tracking.active.axis);
        if (!surface.Valid) return;
        EnsureTargetPlan();
        float target = Mathf.Max(FlatStick, control.stick + surface.outwardSpeed * OutwardResponseInverse);
        control.stick = Move(control.stick, Mathf.Min(target, BridgeRecoveryLimit), StickRiseJerk, StickFallJerk);
        ApplySurfaceAcceleration(surface, 0f, control.stick);
    }

    // Rank 27: EvaluateCurvature
    static float EvaluateCurvature(SlopeFrame frame, float progress)
    {
        float curvature = frame.curvature;
        if (frame.entryCurvature > MinCurvature)
        {
            curvature = Mathf.Lerp(curvature, frame.entryCurvature, 1f - SmoothRange01(progress, 0f, EntryEnd));
        }
        if (frame.exitCurvature > MinCurvature)
        {
            curvature = Mathf.Lerp(curvature, frame.exitCurvature, SmoothRange01(progress, 1f - EntryEnd, 1f));
        }
        return Mathf.Clamp(curvature, MinCurvature, MaxCurvature);
    }

    // Rank 28: PopulateCurvature
    void PopulateCurvature(ref SlopeFrame frame)
    {
        Vector3 entryNormal = SampleConnectedNormal(frame.entry, -frame.axis, frame.collider);
        Vector3 exitNormal = SampleConnectedNormal(frame.exit, frame.axis, frame.collider);
        frame.entryCurvature = entryNormal.sqrMagnitude > Epsilon ? BoundaryCurvature(entryNormal, frame.normal) : 0f;
        frame.exitCurvature = exitNormal.sqrMagnitude > Epsilon ? BoundaryCurvature(frame.normal, exitNormal) : 0f;
        frame.curvature = Mathf.Clamp(Mathf.Max(MinCurvature, Mathf.Max(frame.entryCurvature, frame.exitCurvature)), MinCurvature, MaxCurvature);
    }

    // Rank 29: SampleConnectedNormal
    Vector3 SampleConnectedNormal(Vector3 boundaryPoint, Vector3 outwardAxis, Collider excluded)
    {
        Vector3 sample = boundaryPoint + outwardAxis.normalized * ConnectedProbeOffset;
        RaycastHit[] hits = Physics.RaycastAll(sample + Vector3.up * ForwardProbeHeight, Vector3.down, ForwardProbeDown, groundMask, QueryTriggerInteraction.Ignore);
        float nearest = float.PositiveInfinity;
        Vector3 normal = Vector3.zero;
        for (int i = 0; i < hits.Length; i++)
        {
            RaycastHit hit = hits[i];
            if (hit.collider == null || hit.collider == excluded || Vector3.Distance(hit.point, boundaryPoint) > ConnectedMaxGap || Vector3.Angle(hit.normal, Vector3.up) > MaxSlopeAngle || hit.distance >= nearest)
            {
                continue;
            }
            nearest = hit.distance;
            normal = hit.normal.normalized;
        }
        return normal;
    }

    // Rank 30: ApplyEntryVelocityTransport
    void ApplyEntryVelocityTransport(SlopeFrame frame)
    {
        if (!frame.Valid) return;
        Vector3 normal = frame.normal.normalized;
        Vector3 before = rb.velocity;
        if (Vector3.Dot(before, normal) <= 0.01f || before.sqrMagnitude <= Epsilon)
        {
            return;
        }
        Vector3 slopeVelocity = Vector3.ProjectOnPlane(before, normal);
        if (slopeVelocity.sqrMagnitude <= Epsilon) return;
        rb.AddForce(slopeVelocity.normalized * before.magnitude - before, ForceMode.VelocityChange);
    }

    // Rank 31: SolveAir
    void SolveAir()
    {
        if (!control.released)
        {
            rb.AddForce(direction * AirAcceleration, ForceMode.Acceleration);
        }
    }

    // Rank 32: BridgeRecoveryAcceleration
    float BridgeRecoveryAcceleration()
    {
        SlopeFrame frame = tracking.active;
        if (!frame.Valid || !frame.HasSupport) return 0f;
        Vector3 normal = frame.supportNormal.normalized;
        float planeDistance = Vector3.Dot(rb.worldCenterOfMass - frame.supportPoint, normal);
        float gap = Mathf.Max(0f, Mathf.Abs(planeDistance) - SphereRadius);
        float outward = Mathf.Max(0f, Vector3.Dot(rb.velocity, normal));
        float horizon = Mathf.Max(Time.fixedDeltaTime * 2f, 0.04f);
        float positionRecovery = 2f * (gap + outward * horizon) / Mathf.Max(Epsilon, horizon * horizon);
        return Mathf.Clamp(Mathf.Max(positionRecovery, outward / horizon), 0f, BridgeRecoveryLimit);
    }

    // Rank 33: BlendUnitVectors
    static Vector3 BlendUnitVectors(Vector3 from, Vector3 to, float weight)
    {
        if (from.sqrMagnitude <= Epsilon)
        {
            return to.sqrMagnitude > Epsilon ? to.normalized : Vector3.up;
        }
        if (to.sqrMagnitude <= Epsilon) return from.normalized;
        if (Vector3.Dot(from, to) < 0f) to = -to;
        return Vector3.Slerp(from.normalized, to.normalized, Mathf.Clamp01(weight)).normalized;
    }

    // Rank 34: EntryPreload
    float EntryPreload(SlopeFrame frame, float distance)
    {
        if (!frame.Valid || distance <= 0f) return 0f;
        float speed = Vector3.ProjectOnPlane(rb.velocity, Vector3.up).magnitude;
        float window = Mathf.Max(1f, speed * 0.10f);
        return FlatStick * 0.25f * SmootherStep01(1f - distance / Mathf.Max(0.01f, window));
    }

    // Rank 35: BoundaryCurvature
    static float BoundaryCurvature(Vector3 beforeNormal, Vector3 afterNormal) => Mathf.Clamp(Vector3.Angle(beforeNormal, afterNormal) * Mathf.Deg2Rad / CurvatureDistance, 0f, MaxCurvature);

    // Rank 36: ClearGroundState
    void ClearGroundState()
    {
        tracking.stableDistance = 0f;
        tracking.supportMissDistance = 0f;
        ClearCandidate();
        tracking.active.supportNormal = Vector3.zero;
        bool released = control.released;
        ResetSlopeSession(false);
        control.released = released;
    }

    // Rank 37: ResetSlopeSession
    void ResetSlopeSession(bool clearFrame)
    {
        control.capturedTargetSpeed = -1f;
        control.targetAcceleration = 0f;
        control.stick = 0f;
        control.released = false;
        if (clearFrame) tracking.active = default;
    }

    // Rank 38: SetTravelDirection
    public void SetTravelDirection(Vector3 worldDirection) => direction = NormalizeFlat(worldDirection, direction);

    // Rank 39: ObserverDistanceStep
    float ObserverDistanceStep() => Mathf.Max(rb.velocity.magnitude, ObserverReferenceSpeed) * Time.fixedDeltaTime;

    // Rank 40: SurfaceTravelStep
    float SurfaceTravelStep(SlopeFrame frame) => Mathf.Abs(Vector3.Dot(rb.velocity, frame.axis)) * Time.fixedDeltaTime;

    // Rank 41: RetainedNormal
    Vector3 RetainedNormal() => tracking.active.HasSupport ? tracking.active.supportNormal.normalized : tracking.active.Valid ? tracking.active.normal.normalized : Vector3.up;

    // Rank 42: IncludeProjection
    static void IncludeProjection(Vector3 point, Vector3 axis, ref float min, ref float max)
    {
        float coordinate = Vector3.Dot(point, axis);
        min = Mathf.Min(min, coordinate);
        max = Mathf.Max(max, coordinate);
    }

    // Rank 43: ClearCandidate
    void ClearCandidate()
    {
        tracking.candidateCollider = null;
        tracking.candidateDistance = 0f;
    }
    void Start()
    {
        sub=GameObject.Find("InSubject");
        StartCoroutine(delayStart());
    }
    IEnumerator delayStart()
    {
        yield return new WaitForSeconds(0.8f);
        GameObject arc=GameObject.Find("ArcSlab4_Render");
        sub.transform.position = new Vector3(arc.transform.position.x, arc.transform.position.y + 2, 
            arc.transform.position.z);

    }
    // Rank 44: Awake
    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        direction = NormalizeFlat(travelDirection, transform.forward);
        ResetSlopeSession(true);
    }

    
}
