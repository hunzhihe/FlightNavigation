// Fill out your copyright notice in the Description page of Project Settings.


#include "BanFlightNavMeshBoundsVolume.h"
#include "Engine/CollisionProfile.h"
#include "Components/BrushComponent.h"


ABanFlightNavMeshBoundsVolume::ABanFlightNavMeshBoundsVolume(const FObjectInitializer& ObjectInitializer): Super(ObjectInitializer)
{
	GetBrushComponent()->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	GetBrushComponent()->Mobility = EComponentMobility::Static;

	// Set the volume color 红色
	BrushColor = FColor::Red;

	bColored = true;

#if WITH_EDITORONLY_DATA
	bIsSpatiallyLoaded = false;
#endif
}
