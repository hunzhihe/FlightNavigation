// Fill out your copyright notice in the Description page of Project Settings.


#include "UOctree.h"

FOctreeNode::FOctreeNode(const FVector& InCenter, float InSize)
: Center(InCenter), Extent(FVector(InSize, InSize, InSize)), Size(InSize)
{
}

bool FOctreeNode::Contains(const FVector& Point) const
{
	return FMath::Abs(Point.X - Center.X)<=Extent.X &&
		FMath::Abs(Point.Y - Center.Y)<=Extent.Y &&
		FMath::Abs(Point.Z - Center.Z)<=Extent.Z;
}

void FOctreeNode::Subdivide()
{
	if (!bIsLeaf||Size<100.0f) //最小单元格大小限制
		return;
	
	// 计算子节点尺寸和四分之一尺寸
   // 用于空间分割或层级结构中的尺寸计算
   const float ChildSize = Size * 0.5f;      // 子节点尺寸为原始尺寸的一半
   const float QuarterSize = Size * 0.25f;   // 四分之一尺寸用于更精细的分割计算


	Children.Empty();
	Children.AddDefaulted(8);
	// 子节点
	Children[0] = new FOctreeNode(Center + FVector(-QuarterSize, -QuarterSize, -QuarterSize), ChildSize);
	Children[1] = new FOctreeNode(Center + FVector(QuarterSize, -QuarterSize, -QuarterSize), ChildSize);
	Children[2] = new FOctreeNode(Center + FVector(-QuarterSize, QuarterSize, -QuarterSize), ChildSize);
	Children[3] = new FOctreeNode(Center + FVector(QuarterSize, QuarterSize, -QuarterSize), ChildSize);
	Children[4] = new FOctreeNode(Center + FVector(-QuarterSize, -QuarterSize, QuarterSize), ChildSize);
	Children[5] = new FOctreeNode(Center + FVector(QuarterSize, -QuarterSize, QuarterSize), ChildSize);
	Children[6] = new FOctreeNode(Center + FVector(-QuarterSize, QuarterSize, QuarterSize), ChildSize);
	Children[7] = new FOctreeNode(Center + FVector(QuarterSize, QuarterSize, QuarterSize), ChildSize);
	
	bIsLeaf = false;

	
	 // 遍历所有包含的演员对象，将每个演员分配到合适的子容器中
	 for (auto Actor : ContainedActors)
	 {
		 // 获取当前演员的世界坐标位置
		 FVector ActorLocation = Actor->GetActorLocation();
	 	 // 遍历所有子容器，查找能够包含当前演员位置的容器
	 	 for (auto Child : Children)
	 	 {
	 		 // 检查子容器是否包含演员的位置
	 		 if (Child->Contains(ActorLocation))
	 		 {
	 			 // 将演员插入到合适的子容器中并跳出循环
	 			 Child->InsertActor(Actor);
	 			 break;
	 		 }
	 	 }
	 }
	
}

void FOctreeNode::InsertActor(AActor* Actor)
{
		
	/**
 * 将演员对象插入到八叉树节点中
 * 
 * @param Actor 需要插入的演员对象
 * 
 * 该函数根据节点是否为叶子节点来决定插入逻辑：
 * - 如果是叶子节点，直接添加到当前节点的演员列表中，当演员数量超过8个且节点尺寸大于100时进行细分
 * - 如果不是叶子节点，则找到包含演员位置的子节点并递归插入
 */

	if (bIsLeaf)
	{
		// 叶子节点：直接添加演员到当前节点
		ContainedActors.Add(Actor);
		
		// 当演员数量超过阈值且节点尺寸足够大时，进行节点细分
		if (ContainedActors.Num()>8 && Size >100.0f)
		{
			Subdivide();
		}
	}
	else
	{
		// 非叶子节点：查找合适的子节点进行递归插入
		FVector ActorLocation = Actor->GetActorLocation();
		for ( FOctreeNode* Child : Children)
		{
			if (Child && Child->Contains(ActorLocation))
			{
				Child->InsertActor(Actor);
				break;
			}
		}
	}

}

void FOctreeNode::RemoveActor(AActor* Actor)
{
	if (!Actor)
		return;
	RemoveActorRecursive(Actor);
	MergeIfEmpty();
}

bool FOctreeNode::RemoveActorRecursive(AActor* Actor)
{
	/**
 * 递归地从八叉树节点中移除指定的Actor
 * 
 * @param Actor 要移除的Actor对象指针
 * @return 如果成功移除Actor则返回true，否则返回false
 */
	if (!Actor)
		return false;
    
	// 获取Actor的世界坐标位置
	FVector ActorLocation = Actor->GetActorLocation();
    
	// 如果是叶子节点，直接从包含的Actor列表中移除
	if (bIsLeaf)
	{
		int32 RemovedCount = ContainedActors.Remove(Actor);
		if (RemovedCount > 0)
		{
			ActorCount -= RemovedCount;
			return true;
		}
		return false;
	}
	else
	{
		// 如果是非叶子节点，递归地在子节点中查找并移除Actor
		bool bRemoved = false;
		for (FOctreeNode* Child : Children)
		{
			if (Child->Contains(ActorLocation))
			{
				bRemoved = Child->RemoveActorRecursive(Actor);
				if (bRemoved)
				{
					ActorCount--;
					break;
				}
			}
		}
		return bRemoved;
	}

}

void FOctreeNode::GetAllActors(TArray<AActor*>& OutActors) const
{
	/**
 * 获取所有演员（actors）
 * @param OutActors 输出的演员数组引用，用于存储收集到的所有演员
 */
// 如果是叶子节点，直接将包含的演员添加到输出数组中
if (bIsLeaf)
{
	OutActors.Append(ContainedActors);
}
// 如果不是叶子节点，递归遍历所有子节点获取演员
else
{
	for (const FOctreeNode* Child : Children)
	{
		Child->GetAllActors(OutActors);
	}
}

}

void FOctreeNode::Clear()
{
	ContainedActors.Empty();
	Children.Empty();
	bIsLeaf = true;
	ActorCount = 0;
}

void FOctreeNode::MergeIfEmpty()
{
}

int32 FOctreeNode::GetTotalActorCount() const
{
	return ActorCount;
}

void FOctreeNode::DebugPrint(int32 Depth) const
{
	/**
 * 打印八叉树节点的调试信息
 * 
 * 该函数以递归方式打印当前节点及其子节点的详细信息，用于调试八叉树结构。
 * 输出包括节点中心位置、大小、包含的Actor数量以及是否为叶子节点等信息。
 * 
 * @param Depth 递归深度，用于控制输出缩进，根节点调用时应传入0
 */
	// 生成当前节点的缩进字符串，每个深度层级对应两个空格
	FString Indent;
	for (int32 i = 0; i < Depth; ++i)
	{
		Indent += "  ";
	}
    
	// 打印当前节点的详细信息，包括中心位置、大小、Actor数量和是否为叶子节点
	UE_LOG(LogTemp, Log, TEXT("%sNode: Center(%s), Size: %.2f, Actors: %d, IsLeaf: %s"),
		   *Indent, *Center.ToString(), Size, ActorCount, bIsLeaf ? TEXT("Yes") : TEXT("No"));
    
	// 如果当前节点不是叶子节点，则递归打印所有子节点的调试信息
	if (!bIsLeaf)
	{
		for (const FOctreeNode* Child : Children)
		{
			Child->DebugPrint(Depth + 1);
		}
	}
}

void FOctreeNode::QueryActors(const FVector& Point, float Radius, TArray<AActor*>& OutActors) const
{
	/**
 * 查询指定球形范围内的所有Actor
 * @param Point 查询球形范围的中心点坐标
 * @param Radius 查询球形范围的半径
 * @param OutActors 用于存储查询结果的Actor数组，避免重复添加
 */
	// 计算当前节点的包围盒边界
	FVector MinBounds = Center-Extent;
	FVector MaxBounds = Center+Extent;
	FBox Bounds(MinBounds, MaxBounds);

	// 使用球形与轴对齐包围盒相交检测快速排除不相交的节点
	if (!FMath::SphereAABBIntersection(FVector(Point), static_cast<double>(Radius), Bounds))
	{
		return;
	}

	// 如果是叶子节点，直接检测包含的Actor是否在查询范围内
	if (bIsLeaf)
	{
		for (AActor* Actor : ContainedActors)
		{
			if (Actor && FVector::Dist(Point,Actor->GetActorLocation())<=Radius)
			{
				OutActors.AddUnique(Actor);
			}
		}
	}
	// 如果是非叶子节点，递归查询所有子节点
	else
	{
		for (const FOctreeNode*Child : Children)
		{
			Child->QueryActors(Point, Radius, OutActors);
		}
	}

}
