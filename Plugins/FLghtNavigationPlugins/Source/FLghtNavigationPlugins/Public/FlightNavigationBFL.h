// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "OctreeFlightComponent.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "FlightNavigationBFL.generated.h"

/**
 * 
 */
UCLASS()
class FLGHTNAVIGATIONPLUGINS_API UFlightNavigationBFL : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/*-----------A*算法-----------------*/	
	// 寻路入口函数（寻路主函数）
	UFUNCTION(BlueprintCallable, Category = "FlightNavigation")
	static TArray<FVector> FindPath(
		const FVector& Start,
		const FVector& Goal,
		const TMap<FVector, FAStarNode>& GridNodes,   // 所有网格节点
		float NodeSize = 100.0f                       // 每个格子的尺寸（假设是立方体）
	);
	// 获取当前坐标对应的网格索引（格子中心点）
	static FVector GetGridCenter(const FVector& WorldPos, float NodeSize);

	// 获取相邻的 6 个方向（上下左右前后，可扩展为 26 个）
	static TArray<FVector> GetNeighborGridCenters(const FVector& GridCenter, float NodeSize);

	// 启发函数：3D 欧几里得距离
	static float Heuristic(const FVector& A, const FVector& B);

	static TArray<FVector> RetracePath(
	const TMap<FVector, FAStarNode>& AllNodes,
	const FVector& Start,
	const FVector& Goal);
	/*-----------A*算法-----------------*/


	/*-----------Voxel导航-----------------*/
	// 生成 Voxel 网格：给定范围、体素大小，返回所有可通行 Voxel
	static TMap<FVector,FAStarNode> GenerateVoxelGrid(
		UWorld* World,
		FVector& MinBounds,
		FVector& MaxBounds,
		float VoxelSize
	);
	// 检测某个位置是否可通行（用 LineTrace 向下或全方位）
	static bool IsLocationWalkable(const UWorld* World, const FVector& Location, float VoxelSize);
	/*-----------Voxel导航-----------------*/
};
