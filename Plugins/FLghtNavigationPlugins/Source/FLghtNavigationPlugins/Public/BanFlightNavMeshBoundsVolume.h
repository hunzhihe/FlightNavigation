// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Volume.h"
#include "BanFlightNavMeshBoundsVolume.generated.h"

/**
 * 
 */
UCLASS()
class FLGHTNAVIGATIONPLUGINS_API ABanFlightNavMeshBoundsVolume : public AVolume
{
	GENERATED_BODY()

public:
	explicit ABanFlightNavMeshBoundsVolume(const FObjectInitializer& ObjectInitializer);
};
