// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Volume.h"
#include "AFlightNavMeshBoundsVolume.generated.h"

/**
 * 
 */
UCLASS()
class FLGHTNAVIGATIONPLUGINS_API AFlightNavMeshBoundsVolume : public AVolume
{
	GENERATED_BODY()
public:
	explicit AFlightNavMeshBoundsVolume(const FObjectInitializer& ObjectInitializer);
};
