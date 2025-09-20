// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AFlightNavMeshBoundsVolume.h"
#include "BanFlightNavMeshBoundsVolume.h"
#include "Components/ActorComponent.h"
#include "OctreeFlightComponent.generated.h"

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

// ✅ 定义一个委托类型：当 Voxel 状态变化时触发
// 参数可以是：变化的 Voxel 位置、是否可通行、
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(
	FOnVoxelStateChanged,
	bool, bIsPath
	);

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class FLGHTNAVIGATIONPLUGINS_API UOctreeFlightComponent : public UActorComponent
{
	GENERATED_BODY()
public:
	UOctreeFlightComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightNavigation")
	TSoftObjectPtr<AFlightNavMeshBoundsVolume> FlightNavMeshBoundsVolume =  nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightNavigation")
	TArray<TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>> BanFlightNavMeshBoundsVolumes;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightNavigation")
	float NodeSize = 100.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightNavigation")
	FVector Start = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightNavigation")
	FVector Goal = FVector::ZeroVector;
	
	UPROPERTY(BlueprintReadOnly,Category="FlightNavigation")
	FVector NavMeshMinBounds = FVector::ZeroVector;
	UPROPERTY(BlueprintReadOnly, Category = "FlightNavigation")
	FVector NavMeshMaxBounds = FVector::ZeroVector;

	/**
	 * @brief 寻找飞行路径
	 * 
	 * 使用A*算法在体素网格中寻找从起点到终点的最优飞行路径
	 * @return FVector数组，表示飞行路径上的关键点坐标
	 */
	UFUNCTION(BlueprintCallable, Category = "FlightNavigation")
		TArray<FVector> FindFlightPath();
	
	/**
	 * @brief 初始化并生成飞行导航网格
	 * 
	 * 初始化飞行导航系统所需的体素网格数据，为后续路径规划做准备
	 * 
	 * @return FAStarNode数组，表示初始化后的体素网格节点数据
	 */
	UFUNCTION(BlueprintCallable, Category = "FlightNavigation")
	TMap<FVector, FAStarNode> InitializeGenerateFlightNavMesh();
	
	/**
	 * 更新阻挡盒内的体素状态
	 * 
	 * 该函数用于更新指定区域内的体素阻挡信息，通常用于飞行导航系统中
	 * 动态障碍物的检测和路径规划
	 * 
	 * @param Banbox需要改变内部体素状态的障碍盒
	 * @param bIsBlocked 指示该区域是否被阻挡的布尔值
	 */
	UFUNCTION(BlueprintCallable, Category = "FlightNavigation")
	void UpdateVoxelsInObstructionBox(UPARAM(ref)FVector BanboxCenter, bool bIsBlocked);

    /**
    * 获取禁飞区域包围盒的中心位置
    * 
    * @param BanBox 禁飞区域导航网格边界体积的软对象指针引用
    * @return 返回禁飞区域包围盒的中心世界坐标位置
    */
    UFUNCTION(BlueprintCallable, Category = "FlightNavigation")
    FVector GetBanBoxCenter(UPARAM(ref)TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>& BanBox);


	//可视化被阻塞的区域
	UFUNCTION(BlueprintCallable, Category = "FlightNavigation")
	void DrawDebugVoxelBlocked(const FAStarNode& Voxel);

	//Voxel状态发生改变的委托
	 UPROPERTY(BlueprintAssignable, Category = "FlightNavigation")
	 FOnVoxelStateChanged OnVoxelStateChanged;

	//全体体素网格数据
	UPROPERTY(BlueprintReadOnly, Category = "FlightNavigation")
	TMap<FVector, FAStarNode> VoxelGrids;


	
private:

	TArray<FVector> Path;
	
    //障碍物包围盒体素网格
	TMap<TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>,TArray<FAStarNode*>> VexolinBanVoxelGrids;

	//障碍物包围盒
	TMap<FVector, TSoftObjectPtr<ABanFlightNavMeshBoundsVolume>> BanVoxelGrids;

	// ⭐ 加锁对象：保护 VoxelGrid 的读写
	FCriticalSection VoxelGridCriticalSection;

	//广播函数
	void BroadcastVoxelStateChanged(bool bIsPath);
};
