// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GraphAStar.h"
#include "OctreeFlightComponent.h"
#include "VoxelNavMeshGenerator.generated.h"
/**
 * 
 */

USTRUCT()
struct FVoxel
{
	GENERATED_BODY()

	UPROPERTY()
	FVector Center;      // Voxel 中心点，如 (0,0,0) 表示该格子中心

	UPROPERTY()
	bool bIsWalkable = true; // 是否可通行（可飞行）

	FVoxel(): Center()
	{
	}

	FVoxel(FVector InCenter, bool bWalkable) : Center(InCenter), bIsWalkable(bWalkable) {}
};

/**
 * 负责生成 3D Voxel 导航网格（可飞行体积）
 */

class FLGHTNAVIGATIONPLUGINS_API FVoxelNavMeshGenerator
{
public:
	FVoxelNavMeshGenerator(){};
	~FVoxelNavMeshGenerator(){};
};
