#include "SelectTargetPointUtils.h"
#include "WorldBaseEntity.h"
#include "MapCollisionBox.h"
#include "WorldEntityUtil.h"
#include "TableUtils.h"

static constexpr float EXTRA_MARGIN = 5.0f;
static constexpr float TWO_PI = 2.0f * PI;
static constexpr float ARC_EPS = 1e-4f;
static constexpr float DIST_EPS = 1e-4f;

// ─────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────

static float NormalizeAngleDiff(float A, float B)
{
	float D = FMath::Fmod(A - B + PI, TWO_PI);
	if (D < 0.f) D += TWO_PI;
	return D - PI;
}

// ─────────────────────────────────────────────────────────────────────
// IsAngleBlocked
// ─────────────────────────────────────────────────────────────────────

bool USelectTargetPointUtils::IsAngleBlocked(float A, const TArray<FBlockedArc>& Arcs)
{
	for (const FBlockedArc& Arc : Arcs)
	{
		if (FMath::Abs(NormalizeAngleDiff(A, Arc.CenterAngle)) < Arc.HalfWidth)
		{
			return true;
		}
	}
	return false;
}

// ─────────────────────────────────────────────────────────────────────
// FindBestFreeAngle
// ─────────────────────────────────────────────────────────────────────

bool USelectTargetPointUtils::FindBestFreeAngle(
	float DesiredAngle,
	const TArray<FBlockedArc>& Arcs,
	float& OutAngle)
{
	if (Arcs.Num() == 0)
	{
		OutAngle = DesiredAngle;
		return true;
	}

	if (!IsAngleBlocked(DesiredAngle, Arcs))
	{
		OutAngle = DesiredAngle;
		return true;
	}

	float BestAngle = 0.f;
	float BestDiff = BIG_NUMBER;
	bool bFound = false;

	for (const FBlockedArc& Arc : Arcs)
	{
		for (float Candidate : {Arc.CenterAngle + Arc.HalfWidth + ARC_EPS,
		                        Arc.CenterAngle - Arc.HalfWidth - ARC_EPS})
		{
			if (!IsAngleBlocked(Candidate, Arcs))
			{
				float Diff = FMath::Abs(NormalizeAngleDiff(Candidate, DesiredAngle));
				if (Diff < BestDiff)
				{
					BestDiff = Diff;
					BestAngle = Candidate;
					bFound = true;
				}
			}
		}
	}

	OutAngle = BestAngle;
	return bFound;
}

// ─────────────────────────────────────────────────────────────────────
// ComputeEntityArcs  (law of cosines — exact)
// ─────────────────────────────────────────────────────────────────────

bool USelectTargetPointUtils::ComputeEntityArcs(
	const FVector2D& TargetPos,
	float R,
	float CasterRadius,
	float ExtraMargin,
	const TArray<TPair<FVector2D, float>>& OtherEntities,
	TArray<FBlockedArc>& OutArcs)
{
	for (const auto& Pair : OtherEntities)
	{
		const FVector2D& Pos = Pair.Key;
		float OtherRadius = Pair.Value;
		float MinDist = CasterRadius + OtherRadius + ExtraMargin;
		float D = FVector2D::Distance(Pos, TargetPos);

		if (D < DIST_EPS)
		{
			if (R < MinDist) return false; // fully blocked
			continue;
		}

		float CosVal = (D * D + R * R - MinDist * MinDist) / (2.0f * D * R);
		if (CosVal <= -1.0f) return false; // fully blocked
		if (CosVal >= 1.0f)  continue;     // no blocking

		float HalfAngle = FMath::Acos(FMath::Clamp(CosVal, -1.f, 1.f));
		float CenterAngle = FMath::Atan2(Pos.Y - TargetPos.Y, Pos.X - TargetPos.X);
		OutArcs.Add({CenterAngle, HalfAngle});
	}
	return true; // not fully blocked
}

// ─────────────────────────────────────────────────────────────────────
// ComputeObstacleArcs  (circle vs rounded-rect — analytical)
// ─────────────────────────────────────────────────────────────────────

bool USelectTargetPointUtils::ComputeObstacleArcs(
	const FVector2D& TargetPos,
	float R,
	float CR,
	const FObstacleData2D& Obs,
	TArray<FBlockedArc>& OutArcs)
{
	float HW = Obs.HalfW;
	float HH = Obs.HalfH;
	float HalfDiag = FMath::Sqrt(HW * HW + HH * HH);
	float DObs = FVector2D::Distance(FVector2D(Obs.Center), TargetPos);

	if (DObs - R > HalfDiag + CR)
		return true; // too far, no intersection

	// Transform target pos into obstacle's local space
	float CosA = FMath::Cos(-Obs.AngleRad);
	float SinA = FMath::Sin(-Obs.AngleRad);
	float DX = TargetPos.X - Obs.Center.X;
	float DY = TargetPos.Y - Obs.Center.Y;
	float LTX = DX * CosA - DY * SinA;
	float LTY = DX * SinA + DY * CosA;

	TArray<float> Crossings;

	// ── Side segments: circle-line intersections ──
	auto AddVerticalCrossings = [&](float XVal, float YLo, float YHi)
	{
		float Disc = R * R - FMath::Square(XVal - LTX);
		if (Disc < 0.f) return;
		float SD = FMath::Sqrt(Disc);
		for (float YVal : {LTY + SD, LTY - SD})
		{
			if (YVal >= YLo - DIST_EPS && YVal <= YHi + DIST_EPS)
				Crossings.Add(FMath::Atan2(YVal - LTY, XVal - LTX));
		}
	};
	AddVerticalCrossings( HW + CR, -HH, HH);
	AddVerticalCrossings(-HW - CR, -HH, HH);

	auto AddHorizontalCrossings = [&](float YVal, float XLo, float XHi)
	{
		float Disc = R * R - FMath::Square(YVal - LTY);
		if (Disc < 0.f) return;
		float SD = FMath::Sqrt(Disc);
		for (float XVal : {LTX + SD, LTX - SD})
		{
			if (XVal >= XLo - DIST_EPS && XVal <= XHi + DIST_EPS)
				Crossings.Add(FMath::Atan2(YVal - LTY, XVal - LTX));
		}
	};
	AddHorizontalCrossings( HH + CR, -HW, HW);
	AddHorizontalCrossings(-HH - CR, -HW, HW);

	// ── Corner arcs: circle-circle intersections ──
	struct FCorner { float CX, CY; int QX, QY; };
	const FCorner Corners[] = {
		{ HW,  HH,  1,  1},
		{-HW,  HH, -1,  1},
		{-HW, -HH, -1, -1},
		{ HW, -HH,  1, -1},
	};

	for (const FCorner& C : Corners)
	{
		float DD = FMath::Sqrt(FMath::Square(C.CX - LTX) + FMath::Square(C.CY - LTY));
		if (DD < DIST_EPS) continue;

		float CV = (DD * DD + R * R - CR * CR) / (2.0f * DD * R);
		if (CV < -1.0f || CV > 1.0f) continue;

		float Base = FMath::Atan2(C.CY - LTY, C.CX - LTX);
		float HA = FMath::Acos(FMath::Clamp(CV, -1.f, 1.f));

		for (float Theta : {Base + HA, Base - HA})
		{
			float PX = LTX + R * FMath::Cos(Theta);
			float PY = LTY + R * FMath::Sin(Theta);
			if (C.QX > 0 && PX < HW - DIST_EPS) continue;
			if (C.QX < 0 && PX > -HW + DIST_EPS) continue;
			if (C.QY > 0 && PY < HH - DIST_EPS) continue;
			if (C.QY < 0 && PY > -HH + DIST_EPS) continue;
			Crossings.Add(Theta);
		}
	}

	// ── Classify arcs via midpoint test ──
	auto HitLocal = [&](float Theta) -> bool
	{
		float PX = LTX + R * FMath::Cos(Theta);
		float PY = LTY + R * FMath::Sin(Theta);
		float CLX = FMath::Clamp(PX, -HW, HW);
		float CLY = FMath::Clamp(PY, -HH, HH);
		float DDX = PX - CLX;
		float DDY = PY - CLY;
		return (DDX * DDX + DDY * DDY) < CR * CR;
	};

	if (Crossings.Num() == 0)
	{
		if (HitLocal(0.f))
			return false; // entire circle inside obstacle
		return true;
	}

	Crossings.Sort();

	// Remove near-duplicates
	TArray<float> Filtered;
	Filtered.Add(Crossings[0]);
	for (int32 I = 1; I < Crossings.Num(); ++I)
	{
		if (Crossings[I] - Filtered.Last() > 1e-6f)
			Filtered.Add(Crossings[I]);
	}

	int32 N = Filtered.Num();
	for (int32 I = 0; I < N; ++I)
	{
		float AStart = Filtered[I];
		float AEnd = Filtered[(I + 1) % N];
		if (AEnd <= AStart)
			AEnd += TWO_PI;

		float Mid = AStart + (AEnd - AStart) * 0.5f;
		if (HitLocal(Mid))
		{
			float WStart = AStart + Obs.AngleRad;
			float WEnd   = AEnd   + Obs.AngleRad;
			float ArcCenter = (WStart + WEnd) * 0.5f;
			float ArcHalf   = (WEnd - WStart) * 0.5f;
			OutArcs.Add({ArcCenter, ArcHalf});
		}
	}

	return true;
}

// ─────────────────────────────────────────────────────────────────────
// SolveAtRadius
// ─────────────────────────────────────────────────────────────────────

bool USelectTargetPointUtils::SolveAtRadius(
	const FVector2D& TargetPos,
	float R,
	float CasterRadius,
	float ExtraMargin,
	float DesiredAngle,
	const TArray<TPair<FVector2D, float>>& OtherEntities,
	const TArray<FObstacleData2D>& Obstacles,
	FVector2D& OutPoint,
	TArray<FBlockedArc>& OutBlockedArcs)
{
	OutBlockedArcs.Reset();

	if (!ComputeEntityArcs(TargetPos, R, CasterRadius, ExtraMargin, OtherEntities, OutBlockedArcs))
		return false;

	for (const FObstacleData2D& Obs : Obstacles)
	{
		if (!ComputeObstacleArcs(TargetPos, R, CasterRadius, Obs, OutBlockedArcs))
			return false;
	}

	float BestAngle;
	if (!FindBestFreeAngle(DesiredAngle, OutBlockedArcs, BestAngle))
		return false;

	OutPoint = FVector2D(
		TargetPos.X + R * FMath::Cos(BestAngle),
		TargetPos.Y + R * FMath::Sin(BestAngle));
	return true;
}

// ─────────────────────────────────────────────────────────────────────
// FindValidTargetPointLocation  (public entry point)
// ─────────────────────────────────────────────────────────────────────

FSelectTargetPointResult USelectTargetPointUtils::FindValidTargetPointLocation(
	AWorldBaseEntity* Caster,
	AWorldBaseEntity* Target,
	bool bUseCasterAttackRange)
{
	FSelectTargetPointResult Result;
	if (!Caster || !Target)
		return Result;

	const FVector CasterLoc = Caster->GetActorLocation();
	const FVector TargetLoc = Target->GetActorLocation();
	const FVector2D CasterPos(CasterLoc.X, CasterLoc.Y);
	const FVector2D TargetPos(TargetLoc.X, TargetLoc.Y);

	const float CasterRadius = Caster->GetBoxRadius();
	const float TargetRadius = Target->GetBoxRadius();
	const float MinRadius = CasterRadius + TargetRadius + EXTRA_MARGIN;
	const float Distance = FVector2D::Distance(CasterPos, TargetPos);

	// ── Compute DectionRadius ──
	float DectionRadius = MinRadius;
	if (bUseCasterAttackRange)
	{
		int32 UnitID = Caster->GetBattleSkillInitInfo().UnitID;
		if (TSharedPtr<const GameTable::UnitBase> UnitCfg =
				UTableUtils::GetTableData<GameTable::UnitBase>(Caster, UnitID))
		{
			DectionRadius = FMath::Max(
				FMath::Min(static_cast<float>(UnitCfg->AttackRange), Distance),
				MinRadius);
		}
	}

	if (DectionRadius <= 0.f)
		return Result;

	// ── Ranged: already inside attack range → stay in place ──
	if (bUseCasterAttackRange && DectionRadius > MinRadius && Distance <= DectionRadius)
	{
		Result.bHasValidTargetPoint = true;
		Result.TargetPointLocation = CasterLoc;
		return Result;
	}

	// ── Gather blockers from sphere query ──
	TArray<TPair<FVector2D, float>> OtherEntities;
	TArray<FObstacleData2D> Obstacles;

	float QueryRadius = DectionRadius + CasterRadius + EXTRA_MARGIN;
	TArray<AActor*> ActorList = UWorldEntityUtil::GetActorsInSphere(Target, QueryRadius);
	for (AActor* Actor : ActorList)
	{
		if (Actor == Caster || Actor == Target)
			continue;

		if (AMapCollisionBox* Box = Cast<AMapCollisionBox>(Actor))
		{
			FVector BoxLoc = Box->GetActorLocation();
			FVector BoxSize = Box->GetCollisionBoxSize();
			float YawDeg = Box->GetActorRotation().Yaw;
			Obstacles.Add({
				FVector2D(BoxLoc.X, BoxLoc.Y),
				FMath::DegreesToRadians(YawDeg),
				BoxSize.X * 0.5f,
				BoxSize.Y * 0.5f
			});
			continue;
		}

		if (AWorldBaseEntity* Entity = Cast<AWorldBaseEntity>(Actor))
		{
			FVector Loc = Entity->GetActorLocation();
			OtherEntities.Add({FVector2D(Loc.X, Loc.Y), Entity->GetBoxRadius()});
		}
	}

	// ── Desired approach angle (from Target toward Caster) ──
	float DesiredAngle = FMath::Atan2(CasterPos.Y - TargetPos.Y, CasterPos.X - TargetPos.X);

	FVector2D SolvedPoint;
	TArray<FBlockedArc> Arcs;

	// ── Ranged: try attack circle first ──
	if (bUseCasterAttackRange && DectionRadius > MinRadius)
	{
		if (SolveAtRadius(TargetPos, DectionRadius, CasterRadius, EXTRA_MARGIN,
		                  DesiredAngle, OtherEntities, Obstacles, SolvedPoint, Arcs))
		{
			Result.bHasValidTargetPoint = true;
			Result.TargetPointLocation = FVector(SolvedPoint.X, SolvedPoint.Y, TargetLoc.Z);
			return Result;
		}
		// Attack circle fully blocked → stay in place
		Result.bHasValidTargetPoint = true;
		Result.TargetPointLocation = CasterLoc;
		return Result;
	}

	// ── Melee / fallback: try collision circle ──
	if (SolveAtRadius(TargetPos, MinRadius, CasterRadius, EXTRA_MARGIN,
	                  DesiredAngle, OtherEntities, Obstacles, SolvedPoint, Arcs))
	{
		Result.bHasValidTargetPoint = true;
		Result.TargetPointLocation = FVector(SolvedPoint.X, SolvedPoint.Y, TargetLoc.Z);
		return Result;
	}

	// Fully blocked fallback: return target position
	Result.TargetPointLocation = TargetLoc;
	return Result;
}
