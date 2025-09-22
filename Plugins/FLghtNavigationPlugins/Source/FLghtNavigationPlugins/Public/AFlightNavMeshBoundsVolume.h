// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Volume.h"
#include "AFlightNavMeshBoundsVolume.generated.h"

// 每个网格节点
USTRUCT(BlueprintType)
struct FAStarNode
{
	GENERATED_BODY()

	UPROPERTY()
	FVector Location;// 网格中心位置，如 (0,0,0) 表示该格子中心

	UPROPERTY()
	float NodeSize = 100.0f;

	UPROPERTY()
	bool bIsWalkable = true; // 是否可通行
	
	UPROPERTY()
	float GScore = TNumericLimits<float>::Max(); // 从起点到本节点的实际代价

	UPROPERTY()
	float FScore = TNumericLimits<float>::Max(); // G + H

	//父节点，用于回溯路径
	FAStarNode* Parent = nullptr;
	
	bool operator==(const FAStarNode& Other) const
	{
		return Location == Other.Location;
	}

	friend uint32 GetTypeHash(const FAStarNode& Node)
	{
		return GetTypeHash(Node.Location);
	}
};
/**
 * 
 */
UCLASS()
class FLGHTNAVIGATIONPLUGINS_API AFlightNavMeshBoundsVolume : public AVolume
{
	GENERATED_BODY()
public:
	explicit AFlightNavMeshBoundsVolume(const FObjectInitializer& ObjectInitializer);
};
