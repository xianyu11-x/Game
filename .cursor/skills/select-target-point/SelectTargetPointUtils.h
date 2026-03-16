#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SelectTargetPointUtils.generated.h"

class AWorldBaseEntity;
class AMapCollisionBox;

USTRUCT(BlueprintType)
struct FSelectTargetPointResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly)
	bool bHasValidTargetPoint = false;

	UPROPERTY(BlueprintReadOnly)
	FVector TargetPointLocation = FVector::ZeroVector;
};

struct FBlockedArc
{
	float CenterAngle; // radians
	float HalfWidth;   // radians
};

struct FObstacleData2D
{
	FVector2D Center;
	float AngleRad; // rotation in radians
	float HalfW;
	float HalfH;
};

UCLASS()
class USelectTargetPointUtils : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "Skill|TargetPoint")
	static FSelectTargetPointResult FindValidTargetPointLocation(
		AWorldBaseEntity* Caster,
		AWorldBaseEntity* Target,
		bool bUseCasterAttackRange);

private:
	/** Try to find the best free angle on a circle of radius R centred on TargetPos.
	 *  Returns true and fills OutPoint on success; false when every angle is blocked. */
	static bool SolveAtRadius(
		const FVector2D& TargetPos,
		float R,
		float CasterRadius,
		float ExtraMargin,
		float DesiredAngle,
		const TArray<TPair<FVector2D, float>>& OtherEntities,
		const TArray<FObstacleData2D>& Obstacles,
		FVector2D& OutPoint,
		TArray<FBlockedArc>& OutBlockedArcs);

	/** Compute blocked arcs from circular entities via law of cosines. */
	static bool ComputeEntityArcs(
		const FVector2D& TargetPos,
		float R,
		float CasterRadius,
		float ExtraMargin,
		const TArray<TPair<FVector2D, float>>& OtherEntities,
		TArray<FBlockedArc>& OutArcs);

	/** Compute blocked arcs from a single rotated-rect obstacle via analytical
	 *  circle / rounded-rect intersection. */
	static bool ComputeObstacleArcs(
		const FVector2D& TargetPos,
		float R,
		float CasterRadius,
		const FObstacleData2D& Obs,
		TArray<FBlockedArc>& OutArcs);

	/** Check whether angle A (radians) falls inside any blocked arc. */
	static bool IsAngleBlocked(float A, const TArray<FBlockedArc>& Arcs);

	/** Find the closest free angle to DesiredAngle.  Returns false if all blocked. */
	static bool FindBestFreeAngle(
		float DesiredAngle,
		const TArray<FBlockedArc>& Arcs,
		float& OutAngle);
};
