// Fill out your copyright notice in the Description page of Project Settings.


#include "OctreeFlightComponent.h"

#include "BanFlightNavMeshBoundsVolume.h"
#include "FlightNavigationBFL.h"
#include "MeshPaintVisualize.h"
#include "VoxelNavMeshGenerator.h"


UOctreeFlightComponent::UOctreeFlightComponent()
{
	FlightNavMeshBoundsVolume = nullptr;
}

TArray<FVector> UOctreeFlightComponent::FindFlightPath()
{
	
	Path = UFlightNavigationBFL::FindPath(Start, Goal, VoxelGrids);
	
	for (const FVector& P : Path)
	{
		UE_LOG(LogTemp, Warning, TEXT("Path Point: %s"), *P.ToString());
		DrawDebugSphere(GetWorld(), P, 10, 8, FColor::Red, true, 20, 0);
	}
	return Path;
}

TMap<FVector, FAStarNode> UOctreeFlightComponent::InitializeGenerateFlightNavMesh()
{
	VoxelGrids.Empty();
	if (IsValid(FlightNavMeshBoundsVolume.Get()))
	{
		NavMeshMinBounds = FlightNavMeshBoundsVolume->GetBounds().GetBox().GetCenter()-FlightNavMeshBoundsVolume->GetBounds().GetBox().GetExtent();
		NavMeshMaxBounds = FlightNavMeshBoundsVolume->GetBounds().GetBox().GetCenter()+FlightNavMeshBoundsVolume->GetBounds().GetBox().GetExtent();

		NavMeshMinBounds = FVector (FIntVector(NavMeshMinBounds/NodeSize) * NodeSize);
		NavMeshMaxBounds = FVector (FIntVector(NavMeshMaxBounds/NodeSize) * NodeSize);
	} 
	else
	{
		return TMap<FVector, FAStarNode>();
	}
	
	//FScopeLock Lock(&VoxelGridCriticalSection);
	VoxelGrids = UFlightNavigationBFL::GenerateVoxelGrid(GetWorld(), NavMeshMinBounds, NavMeshMaxBounds, NodeSize);
	
	return VoxelGrids;
}

void UOctreeFlightComponent::UpdateVoxelsInObstructionBox()
{
	BanVoxelGridKey.Empty();
	bool bIsPath = false;
	if (BanFlightNavMeshBoundsVolumes.Num() == 0 || VoxelGrids.Num() == 0 )
	{
		UE_LOG(LogTemp, Warning, TEXT("ObstructionBox is invalid or VoxelGrid is empty."));
		return;
	}

	UFlightNavigationBFL::UpdateVoxelsInAllObstructionBox( GetWorld(),BanFlightNavMeshBoundsVolumes, VoxelGrids, Path, bIsPath, NodeSize);
	
	BroadcastVoxelStateChanged(bIsPath);
}

void UOctreeFlightComponent::DrawDebugVoxelBlocked(const FVector& VoxelCenter)
{
	float VoxelSize = NodeSize;

	DrawDebugBox(GetWorld(), VoxelCenter, FVector(VoxelSize * 0.5f), FQuat::Identity, FColor::Red, false, 100, 0, 1.0f);

}

void UOctreeFlightComponent::BroadcastVoxelStateChanged(bool bIsPath )
{
	// 注意：这个函数可以在 GameThread 或其他线程调用
	// 但 **委托的 Broadcast() 必须在 GameThread 调用！**
	// 所以我们要确保它在 GameThread 执行

	if (IsInGameThread())
	{
		// 在 GameThread 中执行广播
		OnVoxelStateChanged.Broadcast(bIsPath);
	}
	else
	{
		// 如果不是 GameThread，则使用异步任务在 GameThread 中执行广播
		AsyncTask(ENamedThreads::GameThread, [this, bIsPath]()
		{
			OnVoxelStateChanged.Broadcast(bIsPath);
		});
	}
}
