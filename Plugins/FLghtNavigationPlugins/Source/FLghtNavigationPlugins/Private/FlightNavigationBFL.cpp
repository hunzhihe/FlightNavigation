// Fill out your copyright notice in the Description page of Project Settings.


#include "FlightNavigationBFL.h"

#include "Chaos/Deformable/ChaosDeformableCollisionsProxy.h"
#include "Engine/OverlapResult.h"
#include "Kismet/GameplayStatics.h"



TArray<FVector> UFlightNavigationBFL::FindPath(const FVector& Start, const FVector& Goal,
	const TMap<FVector, FAStarNode>& GridNodes, float NodeSize)
{
	// TMap<FVector, FAStarNode> AllNodes = GridNodes;
 //
 //    FVector StartGridCenter = GetGridCenter(Start, NodeSize);
 //    FVector GoalGridCenter = GetGridCenter(Goal, NodeSize);
 //
	// // FVector StartGridCenter = Start;
	// // FVector GoalGridCenter = Goal;
 //
 //    // 查找或创建起点和终点节点
 //    FAStarNode* StartNode = &AllNodes.FindOrAdd(StartGridCenter);
 //    FAStarNode* GoalNode = &AllNodes.FindOrAdd(GoalGridCenter);
 //
 //    StartNode->Location = StartGridCenter;
 //    StartNode->bIsWalkable = AllNodes.Contains(StartGridCenter) ? AllNodes[StartGridCenter].bIsWalkable : true;
 //    StartNode->GScore = 0.0f;
 //    StartNode->FScore = Heuristic(StartGridCenter, GoalGridCenter);
 //
 //    GoalNode->Location = GoalGridCenter;
 //    GoalNode->bIsWalkable = AllNodes.Contains(GoalGridCenter) ? AllNodes[GoalGridCenter].bIsWalkable : true;
 //
 //    TSet<FVector> OpenSet;
 //    TSet<FVector> ClosedSet;
 //
 //    OpenSet.Add(StartGridCenter);
 //
 //    while (!OpenSet.IsEmpty())
 //    {
 //        // 找到 FScore 最小的节点（这里简化，实际可用优先队列）
 //        FVector CurrentGridCenter;
 //        FAStarNode* CurrentNode = nullptr;
 //        float LowestFScore = TNumericLimits<float>::Max();
 //
 //        for (const FVector& GridLoc : OpenSet)
 //        {
 //            const FAStarNode* Node = &AllNodes[GridLoc];
 //            if (Node->FScore < LowestFScore)
 //            {
 //                LowestFScore = Node->FScore;
 //                CurrentGridCenter = GridLoc;
 //                CurrentNode = const_cast<FAStarNode*>(Node);
 //            }
 //        }
 //
 //        if (!CurrentNode || CurrentGridCenter == GoalGridCenter)
 //        {
 //            // 找到路径，开始回溯
 //            if (CurrentGridCenter == GoalGridCenter)
 //            {
 //                return RetracePath(AllNodes, StartGridCenter, GoalGridCenter);
 //            }
 //            break;
 //        }
 //
 //        OpenSet.Remove(CurrentGridCenter);
 //        ClosedSet.Add(CurrentGridCenter);
 //
 //        // 遍历邻居
 //        TArray<FVector> Neighbors = GetNeighborGridCenters(CurrentGridCenter, NodeSize);
 //
 //        for (const FVector& NeighborCenter : Neighbors)
 //        {
 //            if (!AllNodes.Contains(NeighborCenter) || !AllNodes[NeighborCenter].bIsWalkable || ClosedSet.Contains(NeighborCenter))
 //                continue;
 //
 //            float TentativeGScore = CurrentNode->GScore + FVector::Dist(CurrentGridCenter, NeighborCenter);
 //
 //            FAStarNode* NeighborNode = &AllNodes[NeighborCenter];
 //
 //            if (TentativeGScore < NeighborNode->GScore)
 //            {
 //                NeighborNode->Parent = CurrentNode;
 //                NeighborNode->GScore = TentativeGScore;
 //                NeighborNode->FScore = TentativeGScore + Heuristic(NeighborCenter, GoalGridCenter);
 //
 //                if (!OpenSet.Contains(NeighborCenter))
 //                {
 //                    OpenSet.Add(NeighborCenter);
 //                }
 //            }
 //        }
 //    }
 //
 //    return {}; // 没找到路径
	// 复制网格节点（避免修改原始数据）
    TMap<FVector, FAStarNode> AllNodes = GridNodes;

    // 获取网格中心坐标
    const FVector StartGridCenter = GetGridCenter(Start, NodeSize);
    const FVector GoalGridCenter = GetGridCenter(Goal, NodeSize);

    // 查找或创建起点和终点节点
    FAStarNode& StartNode = AllNodes.FindOrAdd(StartGridCenter);
    FAStarNode& GoalNode = AllNodes.FindOrAdd(GoalGridCenter);

    // 初始化起点节点
    StartNode.Location = StartGridCenter;
    StartNode.bIsWalkable = AllNodes.Contains(StartGridCenter) ? AllNodes[StartGridCenter].bIsWalkable : true;
    StartNode.GScore = 0.0f;
    StartNode.FScore = Heuristic(StartGridCenter, GoalGridCenter);

    // 初始化终点节点
    GoalNode.Location = GoalGridCenter;
    GoalNode.bIsWalkable = AllNodes.Contains(GoalGridCenter) ? AllNodes[GoalGridCenter].bIsWalkable : true;

    // 使用优先队列优化开放集
    TArray<FVector> OpenSet;
    TSet<FVector> ClosedSet;

    // 自定义比较函数用于优先队列
    auto FScoreComparator = [&AllNodes](const FVector& A, const FVector& B) {
        return AllNodes[A].FScore < AllNodes[B].FScore;
    };

    OpenSet.HeapPush(StartGridCenter, FScoreComparator);

    while (!OpenSet.IsEmpty())
    {
        // 从优先队列中获取 FScore 最小的节点
        FVector CurrentGridCenter;
        OpenSet.HeapPop(CurrentGridCenter, FScoreComparator, EAllowShrinking::No);

        FAStarNode& CurrentNode = AllNodes[CurrentGridCenter];

        // 检查是否到达目标
        if (CurrentGridCenter.Equals(GoalGridCenter, 1.0f))
        {
            return RetracePath(AllNodes, StartGridCenter, GoalGridCenter);
        }

        ClosedSet.Add(CurrentGridCenter);

        // 获取邻居节点
        TArray<FVector> Neighbors = GetNeighborGridCenters(CurrentGridCenter, NodeSize);

        for (const FVector& NeighborCenter : Neighbors)
        {
            // 跳过无效节点
            if (!AllNodes.Contains(NeighborCenter) || 
                !AllNodes[NeighborCenter].bIsWalkable || 
                ClosedSet.Contains(NeighborCenter))
            {
                continue;
            }

            FAStarNode& NeighborNode = AllNodes[NeighborCenter];
            const float TentativeGScore = CurrentNode.GScore + 
                FVector::Dist(CurrentGridCenter, NeighborCenter);

            // 发现更优路径
            if (TentativeGScore < NeighborNode.GScore)
            {
                NeighborNode.Parent = &CurrentNode;
                NeighborNode.GScore = TentativeGScore;
                NeighborNode.FScore = TentativeGScore + Heuristic(NeighborCenter, GoalGridCenter);

                // 如果不在开放集中，则添加
                if (!OpenSet.Contains(NeighborCenter))
                {
                    OpenSet.HeapPush(NeighborCenter, FScoreComparator);
                }
                else
                {
                    // 如果在开放集中，需要重新调整堆
                    OpenSet.HeapSort(FScoreComparator);
                }
            }
        }
    }

    return TArray<FVector>(); // 没找到路径
}

/**
 * @brief 将世界坐标转换为对应网格单元的中心坐标
 * 
 * 该函数用于计算给定世界坐标所在的网格单元，并返回该网格单元的中心位置。
 * 网格大小由NodeSize参数决定，所有坐标都基于网格对齐。
 * 
 * @param WorldPos 输入的世界坐标位置
 * @param NodeSize 网格单元的大小（边长）
 * @return FVector 返回对应网格单元的中心坐标
 */
FVector UFlightNavigationBFL::GetGridCenter(const FVector& WorldPos, float NodeSize)
{
	// 计算世界坐标在网格中的索引位置
	int32 X = FMath::FloorToInt(WorldPos.X / NodeSize);
	int32 Y = FMath::FloorToInt(WorldPos.Y / NodeSize);
	int32 Z = FMath::FloorToInt(WorldPos.Z / NodeSize);

	// 构建网格索引向量
	FVector GridIndex = FVector(X, Y, Z);
	
	// 计算网格单元的中心坐标：索引位置乘以网格大小，再加上半个网格大小的偏移
	FVector GridCenter = GridIndex * NodeSize + FVector(NodeSize * 0.5f, NodeSize * 0.5f, NodeSize * 0.5f);

	return GridCenter;
}


TArray<FVector> UFlightNavigationBFL::GetNeighborGridCenters(const FVector& GridCenter, float NodeSize)
{
	TArray<FVector> Neighbors;

	int32 X = FMath::FloorToInt(GridCenter.X / NodeSize);
	int32 Y = FMath::FloorToInt(GridCenter.Y / NodeSize);
	int32 Z = FMath::FloorToInt(GridCenter.Z / NodeSize);

	// 26个方向
	TArray<FIntVector> Directions = {
		FIntVector(1, 0, 0),
		FIntVector(0, 1, 0),
		FIntVector(0, 0, 1),
		FIntVector(-1, 0, 0),
		FIntVector(0, -1, 0),
		FIntVector(0, 0, -1),
		
		FIntVector(1, 1, 0),
		FIntVector(1, 0, 1),
		FIntVector(0, 1, 1),
		FIntVector(-1, -1, 0),
		FIntVector(-1, 0, -1),
		FIntVector(0, -1, -1),
		FIntVector(-1, 1, 0),
		FIntVector(-1, 0, 1),
		FIntVector(0, -1, 1),
		FIntVector(1, -1, 0),
		FIntVector(1, 0, -1),
		FIntVector(0, 1, -1),
		
		FIntVector(1, 1, 1),
		FIntVector(-1, -1, -1),
		FIntVector(-1, 1, 1),
		FIntVector(1, -1, 1),
		FIntVector(1, 1, -1),
		FIntVector(-1, 1, -1),
		FIntVector(1, -1, -1),
		FIntVector(-1, -1, 1)
		
	};
	

	for (const FIntVector& Dir : Directions)
	{
		int32 Nx = X + Dir.X;
		int32 Ny = Y + Dir.Y;
		int32 Nz = Z + Dir.Z;

		FVector NeighborCenter = FVector(Nx, Ny, Nz) * NodeSize + FVector(NodeSize * 0.5f);
		Neighbors.Add(NeighborCenter);
	}

	return Neighbors;
}

float UFlightNavigationBFL::Heuristic(const FVector& A, const FVector& B)
{
	// 使用欧几里得距离作为启发函数
	return FVector::Dist(A, B);
}

// 回溯路径
TArray<FVector> UFlightNavigationBFL::RetracePath(
	const TMap<FVector, FAStarNode>& AllNodes,
	const FVector& Start,
	const FVector& Goal)
{
	TArray<FVector> Path;

	FVector CurrentLoc = Goal;
	const FAStarNode* CurrentNode = &AllNodes.FindChecked(CurrentLoc);

	while (CurrentNode && CurrentNode->Location != Start)
	{
		Path.Add(CurrentNode->Location);
		CurrentNode = CurrentNode->Parent;
		if (!CurrentNode) break;
	}

	if (CurrentNode && CurrentNode->Location == Start)
	{
		Path.Add(Start); // 可选
	}

	Algo::Reverse(Path);
	return Path;
}

TMap<FVector,FAStarNode> UFlightNavigationBFL::GenerateVoxelGrid(UWorld* World,  FVector& MinBounds,
	 FVector& MaxBounds, float VoxelSize)
{
	TMap<FVector, FAStarNode> VoxelGrids;
	for (float X = MinBounds.X; X < MaxBounds.X; X += VoxelSize)
	{
		for (float Y = MinBounds.Y; Y < MaxBounds.Y; Y += VoxelSize)
		{
			for (float Z = MinBounds.Z; Z < MaxBounds.Z; Z += VoxelSize)
			{
				FVector VoxelCenter = FVector(X + VoxelSize * 0.5f, Y + VoxelSize * 0.5f, Z + VoxelSize * 0.5f);
				bool bWalkable = IsLocationWalkable(World, VoxelCenter, VoxelSize);

				VoxelGrids.Add(VoxelCenter, FAStarNode(VoxelCenter, VoxelSize, bWalkable));

				// 可选：可视化每个体素（调试用）
				
				FColor Color = bWalkable ? FColor::Green : FColor::Red;
				DrawDebugBox(World, VoxelCenter, FVector(VoxelSize * 0.5f), FQuat::Identity, Color, false, 50, 0, 1.0f);
				
			}
		}
	}
	return VoxelGrids;
}

bool UFlightNavigationBFL::IsLocationWalkable( const UWorld* World, const FVector& Location, float VoxelSize)
{
	if (!World) return false;

	// 定义检测用的盒体范围
	float BoxHalfExtent = VoxelSize * 0.5f; // 盒体的一半长度，比如 VoxelSize=100cm → 盒体是 50x50x50 cm
	FVector BoxCenter = Location;           // 盒体中心点

	// 可根据需求调整盒体的大小，比如：
	// FVector BoxExtent(BoxHalfExtent, BoxHalfExtent, BoxHalfExtent); // 正方体
	// 或者更矮一些的盒子，比如只检测脚下和身体周围：
	// FVector BoxExtent(BoxHalfExtent, BoxHalfExtent, 20.0f);

	FCollisionShape BoxShape = FCollisionShape::MakeBox(FVector(BoxHalfExtent, BoxHalfExtent, BoxHalfExtent));
	// 或者更矮的盒子（比如只检测角色脚下一小块，而不是全身）：
	// FCollisionShape BoxShape = FCollisionShape::MakeBox(FVector(BoxHalfExtent, BoxHalfExtent, 30.0f));

	// 设置碰撞查询参数
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(nullptr); // 可忽略特定 Actor，比如自己

	// 碰撞类型
	ECollisionChannel TraceChannel = ECC_Visibility; // 或者你自定义的可行走/障碍物通道，比如 ECC_GameTraceChannel1

	TArray<FOverlapResult> Overlaps;
	bool bBlockingHit = World->OverlapMultiByChannel(
		Overlaps,
		BoxCenter,
		FQuat::Identity,         // 无旋转
		TraceChannel,            // 碰撞通道
		BoxShape,                // 盒体形状
		Params
	);

	// 可选：绘制调试盒（开发时可见）
	// DrawDebugBox(World, BoxCenter, FVector(BoxHalfExtent, BoxHalfExtent, BoxHalfExtent), FQuat::Identity, FColor::Green, false, 2.0f);

	// 如果有任何一个物体 **阻挡（Blocking）** 了这个盒体区域，那么认为不可行走
	for (const FOverlapResult& Overlap : Overlaps)
	{
		if (Overlap.bBlockingHit)
		{
			// 如果需要，可以进一步判断 Overlap.GetActor() 是什么，比如忽略某些类型
			return false;
		}
	}

	// 没有检测到阻挡物，认为可行走
	return true;
	
}

void UFlightNavigationBFL::UpdateVoxelsInAllObstructionBox(
    const UWorld* World,
	TArray<TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>>& BanFlightNavMeshBoundsVolumes,
	TMap<TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>,TArray<FAStarNode*>>& VexolinBanVoxelGrids,
	TMap<FVector, FAStarNode>& VoxelGrids,
	float NodeSize)
{
	//TMap<TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>,TArray<FAStarNode>> VexolinBanVoxelGrids;
	
	for ( auto& ObstructionBox : BanFlightNavMeshBoundsVolumes)
	{
		//包围盒中所有的体素
		TArray<FAStarNode*> BanVoxelGridS;
		// 障碍物 Box 的最小/最大点
		FVector BoxMin = ObstructionBox->GetBounds().GetBox().GetCenter()-ObstructionBox->GetBounds().GetBox().GetExtent();
		FVector BoxMax = ObstructionBox->GetBounds().GetBox().GetCenter()+ObstructionBox->GetBounds().GetBox().GetExtent();
		
		for (auto& BanVoxel: VoxelGrids)
		{
			if (ObstructionBox->GetBounds().GetBox().IsInside(BanVoxel.Key))
			{
                BanVoxelGridS.Add(&BanVoxel.Value);
				
				VoxelGrids[BanVoxel.Key].bIsWalkable = false;
	                	
				DrawDebugVoxelBlocked( World,BanVoxel.Key, NodeSize);
			}
		}
		VexolinBanVoxelGrids.Add(ObstructionBox,BanVoxelGridS);
		// });
	}
}

void UFlightNavigationBFL::DrawDebugVoxelBlocked(const UWorld* World,const FVector& VoxelCenter, float NodeSize)
{
	float VoxelSize = NodeSize;

	DrawDebugBox(World, VoxelCenter, FVector(VoxelSize * 0.5f), FQuat::Identity, FColor::Red, false, 100, 0, 1.0f);
}




