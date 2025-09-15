// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UOctree.generated.h"


/**
 * \struct FObstacleInfo
 * \brief 用于表示场景中障碍物的信息，包括其包围盒中心、范围和半径。
 * 
 * 此结构体用于存储一个Actor的边界信息，并提供检测点是否在障碍物内、
 * 是否与其他包围盒或球体相交的功能。
 */
USTRUCT(BlueprintType)
struct FObstacleInfo
{
	GENERATED_BODY()

	/**
	 * \brief 指向该障碍物对应的Actor对象。
	 */
	UPROPERTY()
	AActor* Actor = nullptr;

	/**
	 * \brief 障碍物包围盒的中心坐标。
	 */
	FVector Center;

	/**
	 * \brief 障碍物包围盒在各轴上的延伸距离（从中心到边的距离）。
	 */
	FVector Extent;

	/**
	 * \brief 障碍物包围盒的等效球体半径（Extent的模长）。
	 */
	float Radius;

	/**
	 * \brief 默认构造函数，初始化所有成员为默认值。
	 */
	FObstacleInfo() : Actor(nullptr), Center(FVector::ZeroVector), Extent(FVector::ZeroVector), Radius(0.0f) {}

	/**
	 * \brief 带参构造函数，根据传入的Actor计算其包围盒信息。
	 * 
	 * \param InActor 要计算包围盒信息的Actor指针。
	 */
	FObstacleInfo(AActor* InActor) : Actor(InActor)
	{
		if (Actor)
		{
			// 获取Actor的所有组件构成的包围盒
			FBox BoundingBox = Actor->GetComponentsBoundingBox(true);
			Center = BoundingBox.GetCenter();   // 获取包围盒中心
			Extent = BoundingBox.GetExtent();   // 获取包围盒延伸量
			Radius = Extent.Size();             // 计算等效球体半径
		}
	}

	/**
	 * \brief 判断给定点是否在当前障碍物包围盒内（支持容差）。
	 * 
	 * \param Point     待检测的点坐标。
	 * \param Tolerance 容差值，用于扩大检测范围。
	 * \return 如果点在包围盒内（含容差），返回true；否则返回false。
	 */
	bool Contains(const FVector& Point, float Tolerance = 0.0f) const
	{
		return FMath::Abs(Point.X - Center.X) <= Extent.X + Tolerance &&
			   FMath::Abs(Point.Y - Center.Y) <= Extent.Y + Tolerance &&
			   FMath::Abs(Point.Z - Center.Z) <= Extent.Z + Tolerance;
	}

	/**
	 * \brief 判断当前障碍物包围盒是否与另一个包围盒相交。
	 * 
	 * 使用轴对齐包围盒(AABB)的相交检测方法。
	 * 
	 * \param OtherCenter 另一个包围盒的中心坐标。
	 * \param OtherExtent 另一个包围盒的延伸量。
	 * \return 如果两个包围盒相交，返回true；否则返回false。
	 */
	bool Intersects(const FVector& OtherCenter, const FVector& OtherExtent) const
	{
		return FMath::Abs(Center.X - OtherCenter.X) <= (Extent.X + OtherExtent.X) &&
			   FMath::Abs(Center.Y - OtherCenter.Y) <= (Extent.Y + OtherExtent.Y) &&
			   FMath::Abs(Center.Z - OtherCenter.Z) <= (Extent.Z + OtherExtent.Z);
	}

	/**
	 * \brief 判断当前障碍物包围盒是否与指定球体相交。
	 * 
	 * 方法是找到球心在包围盒上最近的点，然后比较该点与球心之间的距离是否小于等于球体半径。
	 * 
	 * \param SphereCenter 球体的中心坐标。
	 * \param SphereRadius 球体的半径。
	 * \return 如果包围盒与球体相交，返回true；否则返回false。
	 */
	bool IntersectsSphere(const FVector& SphereCenter, float SphereRadius) const
	{
		// 找到球心在AABB上最近的点
		FVector ClosestPoint;
		ClosestPoint.X = FMath::Clamp(SphereCenter.X, Center.X - Extent.X, Center.X + Extent.X);
		ClosestPoint.Y = FMath::Clamp(SphereCenter.Y, Center.Y - Extent.Y, Center.Y + Extent.Y);
		ClosestPoint.Z = FMath::Clamp(SphereCenter.Z, Center.Z - Extent.Z, Center.Z + Extent.Z);
        
		// 检查球心到最近点的距离平方是否小于等于球体半径的平方
		return FVector::DistSquared(SphereCenter, ClosestPoint) <= FMath::Square(SphereRadius);
	}
};


/**
 * 
 */
/**
 * 八叉树节点结构体
 * 用于空间分割的数据结构，将3D空间递归划分为八个子区域
 */
USTRUCT(BlueprintType)
struct FOctreeNode
{
	GENERATED_BODY()
	
	/** 节点包围盒的中心点坐标 */
	FVector Center;
	
	/** 节点包围盒的范围（从中心到各面的距离） */
	FVector Extent;

	/** 节点的尺寸 */
	float Size;
	
	/** 子节点数组，最多包含8个子节点 */
	TArray<FOctreeNode*> Children;
	
	/** 当前节点包含的Actor对象列表 */
	TArray<AActor*> ContainedActors;
	
	/** 标记节点是否为叶子节点 */
	bool bIsLeaf =  true;

	int32 ActorCount = 0;

	/**
	 * 默认构造函数
	 * 初始化中心点和范围都为零向量
	 */
	FOctreeNode() : Center(FVector::ZeroVector), Extent(FVector::ZeroVector),Size(0.0f) {}
	
	/**
	 * 带参数构造函数
	 * @param InCenter 节点包围盒的中心点坐标
	 * @param InExtent
	 * @param InSize
	 */
	FOctreeNode( const FVector& InCenter,float InSize); 
    
/**
 * 检查指定点是否在当前区域内
 * @param Point 要检查的三维坐标点
 * @return 如果点在区域内返回true，否则返回false
 */
	bool Contains(const FVector& Point) const;

/**
 * 将当前区域细分为更小的子区域
 * 通常用于空间分割数据结构中，将一个大的区域分割成多个小区域以提高查询效率
 */
	void Subdivide();

/**
 * 向当前区域插入一个游戏对象
 * @param Actor 要插入的游戏对象指针
 */
	void InsertActor(AActor* Actor);

	/**
 * 查询指定范围内的所有游戏对象
 * @param Point 查询范围的中心点坐标
 * @param Radius 查询范围的半径
 * @param OutActors 用于存储查询结果的游戏对象数组引用
 */
	void QueryActors(const FVector& Point, float Radius, TArray<AActor*>& OutActors) const;

	/**
 * 从当前区域移除一个游戏对象
 * @param Actor 要移除的游戏对象指针
 */
	void RemoveActor(AActor* Actor);

/**
 * 递归移除指定的Actor及其子Actor
 * @param Actor 要移除的Actor对象指针
 * @return 如果成功移除返回true，否则返回false
 */
bool RemoveActorRecursive(AActor* Actor);

/**
 * 获取所有Actor对象并存储到输出数组中
 * @param OutActors 用于存储所有Actor对象的输出数组引用
 */
void GetAllActors(TArray<AActor*>& OutActors) const;

/**
 * 清空所有Actor数据
 */
void Clear();

/**
 * 如果容器为空则执行合并操作
 */
void MergeIfEmpty();

/**
 * 获取总的Actor数量
 * @return 返回当前存储的Actor总数
 */
int32 GetTotalActorCount() const;

/**
 * 打印调试信息
 * @param Depth 调试信息的缩进深度，默认为0
 */
void DebugPrint(int32 Depth = 0) const;

};

// UCLASS(BlueprintType)
// class FLGHTNAVIGATIONPLUGINS_API UUOctree : public UObject
// {
// 	GENERATED_BODY()
//
// public:
// 	UUOctree();
//
// 	//初始化八叉树
// 	UFUNCTION(BlueprintCallable, Category = "Octree")
// 	void Initialize(const FVector& Origin, const FVector& Size, int32 MaxDepth = 8);
//
// 	
// };
