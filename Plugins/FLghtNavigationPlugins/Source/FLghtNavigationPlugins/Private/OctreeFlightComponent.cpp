// Fill out your copyright notice in the Description page of Project Settings.


#include "OctreeFlightComponent.h"
#include "DrawDebugHelpers.h"
#include "Async/Async.h"
#include "BanFlightNavMeshBoundsVolume.h"
#include "FlightNavigationBFL.h"


UOctreeFlightComponent::UOctreeFlightComponent()
{
	FlightNavMeshBoundsVolume = nullptr;
}

TArray<FVector> UOctreeFlightComponent::FindFlightPath()
{
	
	Path = UFlightNavigationBFL::FindPath(Start, Goal, VoxelGrids);
    #if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	for (const FVector& P : Path)
	{
       
		UE_LOG(LogTemp, Warning, TEXT("Path Point: %s"), *P.ToString());
		DrawDebugSphere(GetWorld(), P, 10, 8, FColor::Red, false, 5, 0);
	}
    #endif
	return Path;
}

TMap<FVector, FAStarNode> UOctreeFlightComponent::InitializeGenerateFlightNavMesh()
{
	VoxelGrids.Empty();
	if (IsValid(FlightNavMeshBoundsVolume.Get()))
	{
		 NavMeshMinBounds = FlightNavMeshBoundsVolume->GetBounds().GetBox().GetCenter()-FlightNavMeshBoundsVolume->GetBounds().GetBox().GetExtent();
		 NavMeshMaxBounds = FlightNavMeshBoundsVolume->GetBounds().GetBox().GetCenter()+FlightNavMeshBoundsVolume->GetBounds().GetBox().GetExtent();
		//
		NavMeshMinBounds = FVector (FIntVector(NavMeshMinBounds/NodeSize) * NodeSize);
		NavMeshMaxBounds = FVector (FIntVector(NavMeshMaxBounds/NodeSize) * NodeSize);
		VoxelGrids = UFlightNavigationBFL::GenerateVoxelGrid(GetWorld(), NavMeshMinBounds, NavMeshMaxBounds, NodeSize);
	} 
	else
	{
		return TMap<FVector, FAStarNode>();
	}

	if (BanFlightNavMeshBoundsVolumes.Num() > 0)
	{
		// 获取所有禁止导航的障碍包围盒
		for (auto& Volume : BanFlightNavMeshBoundsVolumes)
		{
			if (IsValid(Volume.Get()))
			{
				BanVoxelGrids.Add(Volume.Get()->GetBounds().GetBox().GetCenter(),Volume);
			}
		}
		
		UFlightNavigationBFL::UpdateVoxelsInAllObstructionBox( GetWorld(),BanFlightNavMeshBoundsVolumes,VexolinBanVoxelGrids, VoxelGrids, NodeSize);
	}
	
	return VoxelGrids;
}

void UOctreeFlightComponent::UpdateVoxelsInObstructionBox(FVector BanboxCenter, bool bIsBlocked)
{
	bool bIsPath = false;
	if (BanFlightNavMeshBoundsVolumes.Num() == 0 || VoxelGrids.Num() == 0 )
	{
		UE_LOG(LogTemp, Warning, TEXT("ObstructionBox is invalid or VoxelGrid is empty."));
		return;
	}
	TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>& Banbox = *BanVoxelGrids.Find(BanboxCenter);
	
	for (FAStarNode* BanVoxel : *VexolinBanVoxelGrids.Find(Banbox))
	{
		BanVoxel->bIsWalkable = bIsBlocked;
		if (Path.Num()>0)
		{
			for (FVector P : Path)
			{
				if (P == BanVoxel->Location)
				{
					bIsPath = true;
				}
			}
		}
	}
	
	BroadcastVoxelStateChanged(bIsPath);
}

FVector UOctreeFlightComponent::GetBanBoxCenter(TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>& BanBox)
{
	return BanBox.Get()->GetBounds().GetBox().GetCenter();
}

void UOctreeFlightComponent::DrawDebugVoxelBlocked(const FAStarNode& Voxel)
{
	float VoxelSize = Voxel.NodeSize;
	FVector VoxelCenter = Voxel.Location;

	//DrawDebugBox(GetWorld(), VoxelCenter, FVector(VoxelSize * 0.5f), FQuat::Identity, FColor::Red, false, 100, 0, 1.0f);

	bool bWalkable = Voxel.bIsWalkable;
	FColor Color = bWalkable ? FColor::Green : FColor::Red;
	DrawDebugBox(GetWorld(), VoxelCenter, FVector(VoxelSize * 0.5f), FQuat::Identity, Color, false, 50, 0, 1.0f);
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
