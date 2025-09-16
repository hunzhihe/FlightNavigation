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

TArray<FVector> UOctreeFlightComponent::FindFlightPath(const TArray<FAStarNode>& VoxelGrid)
{
	// TMap<FVector, FAStarNode> GridNodes;
	//
	// TArray<FAStarNode> AllNodes = VoxelGrid;
	//
	// for (FAStarNode& Node : AllNodes)
	// {
	// 	GridNodes.Add(Node.Location, Node);
	// }

	FScopeLock Lock(&VoxelGridCriticalSection);
	
	TArray<FVector> Path = UFlightNavigationBFL::FindPath(Start, Goal, VoxelGrids);
	
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
		
	} 
	else
	{
		return TMap<FVector, FAStarNode>();
	}
	
	FScopeLock Lock(&VoxelGridCriticalSection);
	VoxelGrids = UFlightNavigationBFL::GenerateVoxelGrid(GetWorld(), NavMeshMinBounds, NavMeshMaxBounds, NodeSize);
	
	return VoxelGrids;
}

void UOctreeFlightComponent::UpdateVoxelsInObstructionBox(const TArray<FAStarNode>& VoxelGrid)
{
	if (BanFlightNavMeshBoundsVolumes.Num() == 0 || VoxelGrid.Num() == 0 )
	{
		UE_LOG(LogTemp, Warning, TEXT("ObstructionBox is invalid or VoxelGrid is empty."));
		return;
	}
	
	const float VoxelSize = NodeSize;
    //需要更新的 Voxel
     
	for ( auto ObstructionBox : BanFlightNavMeshBoundsVolumes)
	{
		  // 障碍物 Box 的最小/最大点
          FVector BoxMin = ObstructionBox->GetBounds().GetBox().GetCenter()-ObstructionBox->GetBounds().GetBox().GetExtent();
          FVector BoxMax = ObstructionBox->GetBounds().GetBox().GetCenter()+ObstructionBox->GetBounds().GetBox().GetExtent();

		 //生成该 Box 内的 Voxel
		 //BanVoxelGrid = UFlightNavigationBFL::GenerateVoxelGrid(GetWorld(), BoxMin, BoxMax, NodeSize); 

         // 计算该 Box 覆盖的 Voxel 范围（按 VoxelSize 对齐的格子索引）
         int32 MinX = FMath::FloorToInt(BoxMin.X / VoxelSize);
         int32 MinY = FMath::FloorToInt(BoxMin.Y / VoxelSize);
         int32 MinZ = FMath::FloorToInt(BoxMin.Z / VoxelSize);
         
         int32 MaxX = FMath::FloorToInt(BoxMax.X / VoxelSize);
         int32 MaxY = FMath::FloorToInt(BoxMax.Y / VoxelSize);
         int32 MaxZ = FMath::FloorToInt(BoxMax.Z / VoxelSize);

        UE_LOG(LogTemp, Log, TEXT("Updating blocked Voxels from (%d,%d,%d) to (%d,%d,%d)"), 
        MinX, MinY, MinZ, MaxX, MaxY, MaxZ);
        // 遍历该范围内的所有 Voxel 索引
        for (int32 X = MinX; X < MaxX; ++X)
        {
             for (int32 Y = MinY; Y < MaxY; ++Y)
             {
                 for (int32 Z = MinZ; Z < MaxZ; ++Z)
                 {
                     // 计算该 Voxel 的中心世界坐标
                     FVector VoxelCenter(
                    (X+0.5 ) * VoxelSize,
                    (Y+0.5 ) * VoxelSize,
                    (Z+0.5 ) * VoxelSize
                     );
                 	//  Voxel 的索引键（FIntVector）
                 	BanVoxelGridKey.Add(VoxelCenter);
                 }
             }
        }
	}
	//异步，在GameThread中实际更新VoxelGrid(线程安全)
	AsyncTask(ENamedThreads::GameThread, [this]()
	{
		// 锁
		FScopeLock Lock(&VoxelGridCriticalSection);
		
		for (auto VoxelCenter : BanVoxelGridKey)
		{
				// 可选：检查该 VoxelCenter 是否真的在障碍物 Box 内（更精确，但通常索引已足够）
				// if (ObstructionBox->GetBounds().GetBox().IsInside(VoxelCenter))
				// {
					// 如果 VoxelGrid 中存在该 Voxel，则设置其为不可通行
					if (VoxelGrids.Contains(VoxelCenter))
					{
						VoxelGrids[VoxelCenter].bIsWalkable = false;
						#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
						// 可选：调试绘制（仅在开发时）
						DrawDebugVoxelBlocked(VoxelCenter);
						#endif
					}
		}
	});
}

void UOctreeFlightComponent::DrawDebugVoxelBlocked(const FVector& VoxelCenter)
{
	float VoxelSize = NodeSize;

	DrawDebugBox(GetWorld(), VoxelCenter, FVector(VoxelSize * 0.5f), FQuat::Identity, FColor::Red, false, 100, 0, 1.0f);

}
