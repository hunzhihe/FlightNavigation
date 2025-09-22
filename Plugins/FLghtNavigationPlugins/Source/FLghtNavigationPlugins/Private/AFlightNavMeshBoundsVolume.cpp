// Fill out your copyright notice in the Description page of Project Settings.


#include "AFlightNavMeshBoundsVolume.h"
#include "Engine/CollisionProfile.h"
#include "Components/BrushComponent.h"



AFlightNavMeshBoundsVolume::AFlightNavMeshBoundsVolume(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	GetBrushComponent()->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	GetBrushComponent()->Mobility = EComponentMobility::Static;

	BrushColor = FColor(200, 200, 200, 255);

	bColored = true;

#if WITH_EDITORONLY_DATA
	bIsSpatiallyLoaded = false;
#endif
}
