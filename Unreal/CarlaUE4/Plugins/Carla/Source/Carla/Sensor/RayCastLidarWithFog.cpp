// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/RayCastLidarWithFog.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

FActorDefinition ARayCastLidarWithFog::GetSensorDefinition()
{
	return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_with_fog"));
}

ARayCastLidarWithFog::ARayCastLidarWithFog(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
	SetSeed(Description.RandomSeed);
}

void ARayCastLidarWithFog::Set(const FActorDescription& ActorDescription)
{
	ASensor::Set(ActorDescription);
	FLidarDescription LidarDescription;
	UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
	Set(LidarDescription);
}

void ARayCastLidarWithFog::Set(const FLidarDescription& LidarDescription)
{
	Description = LidarDescription;
	LidarData = FLidarData(Description.Channels);
	CreateLasers();
	PointsPerChannel.resize(Description.Channels);

	// Compute drop off model parameters
	DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
	DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
	DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

void ARayCastLidarWithFog::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidarWithFog::PostPhysTick);
	SimulateLidar(DeltaTime);

	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
		auto DataStream = GetDataStream(*this);
		DataStream.Send(*this, LidarData, DataStream.PopBufferFromPool());
	}
}

// float ARayCastLidarWithFog::ComputeIntensity(const FSemanticDetection& RawDetection) const
// {
// 	const carla::geom::Location HitPoint = RawDetection.point;
// 	const float Distance = HitPoint.Length();

// 	const float AttenAtm = Description.AtmospAttenRate;
// 	const float AbsAtm = exp(-AttenAtm * Distance);

// 	const float IntRec = AbsAtm;

// 	return IntRec;
// }

ARayCastLidarWithFog::FDetection ARayCastLidarWithFog::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
{
	FDetection Detection;

	const FVector HitPoint = HitInfo.ImpactPoint;
	Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

	const float Distance = Detection.point.Length();

	const float AttenAtm = Description.AtmospAttenRate;
	const float AbsAtm = exp(-AttenAtm * Distance);

	float OriginalIntensity = static_cast<uint32_t>(AbsAtm * 255);

	FWeatherParameters weather = GetEpisode().GetWeather()->GetCurrentWeather();
	// uint32_t is_modified = 0;
	float FogDensity = weather.FogDensity;
	const std::string AlphaKey = GetAlphaByFogDensity(FogDensity);

	if (FogDensity < 0)
	{
		FogDensity = 0;
	}
	else if (FogDensity > 100)
	{
		FogDensity = 100;
	}


	// Fog density changed
	if (CacheAlpha != AlphaKey)
	{
		GetStepSizeData(AlphaKey);
		CacheAlpha = AlphaKey;
	}

	if (FogDensity > 0)
	{
		// hard
		float Alpha = std::stof(AlphaKey);
		float HardIntRec = std::round(exp(-2 * Alpha * Distance) * OriginalIntensity);

		// soft
		// ParameterSet
		float Mor = std::log(20) / Alpha;
		float Beta = 0.046f / Mor;
		float Gamma = 0.000001f;

		const AActor* HitActor = HitInfo.Actor.Get();
		const FActorRegistry& Registry = GetEpisode().GetActorRegistry();
		if (HitActor != nullptr)
		{
			const FCarlaActor* view = Registry.FindCarlaActor(HitActor);
			// if (view)
			// {
			// 	Detection.actor_type = static_cast<uint32_t>(view->GetActorType());
			// }
			Gamma = GetReflectivityFromHitResult(HitInfo) / std::pow(10, 5);
		}

		float Beta0 = Gamma / M_PI;
		float RoundedIntRec = static_cast<int>(std::round(Distance * 10)) / 10.0;
		char buffer[20];
		std::snprintf(buffer, sizeof(buffer), "%.1f", RoundedIntRec);
		std::string Key(buffer);

		// R_tmp, i_soft
		const std::vector<std::string>& Data = StepSizeData.at(Key);
		float StepDataDistance = std::stof(Data[0]);
		float StepDataIntRec = std::stof(Data[1]);

		StepDataIntRec = StepDataIntRec * OriginalIntensity * std::pow(Distance, 2.0) * Beta / Beta0;

		// i_soft
		if (StepDataIntRec > 255)
		{
			StepDataIntRec = 255;
		}

		Detection.intensity = HardIntRec;

		if (StepDataIntRec > HardIntRec)
		{
			float ScalingFactor = StepDataDistance / Distance;
			// noise
			float Noise = 10.0f;
			float DistanceNoise = RandomEngine->GetUniformFloatInRange(Distance - Noise, Distance + Noise);
			float NoiseFactor = Distance / DistanceNoise;
			float TotalScaling = ScalingFactor * NoiseFactor;

			Detection.point.x *= TotalScaling;
			Detection.point.y *= TotalScaling;
			Detection.point.z *= TotalScaling;
			Detection.intensity = StepDataIntRec;
			// is_modified = 1;
		}
	}
	else
	{
		Detection.intensity = OriginalIntensity;
	}
	Detection.point.y *= -1;
	// Detection.original_intensity = OriginalIntensity;
	// Detection.is_modified = is_modified;

	return Detection;
}

void ARayCastLidarWithFog::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel)
{
	Super::PreprocessRays(Channels, MaxPointsPerChannel);

	for (auto ch = 0u; ch < Channels; ch++)
	{
		for (auto p = 0u; p < MaxPointsPerChannel; p++)
		{
			RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
		}
	}
}

bool ARayCastLidarWithFog::PostprocessDetection(FDetection& Detection) const
{
	if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon())
	{
		const auto ForwardVector = Detection.point.MakeUnitVector();
		const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
		Detection.point += Noise;
	}

	const float Intensity = Detection.intensity / 255;
	// const float Intensity = Detection.intensity;
	if (Intensity > Description.DropOffIntensityLimit)
		return true;
	else
		return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
}

void ARayCastLidarWithFog::ComputeAndSaveDetections(const FTransform& SensorTransform)
{
	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
		PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();

	LidarData.ResetMemory(PointsPerChannel);

	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
	{
		for (auto& hit : RecordedHits[idxChannel])
		{
			FDetection Detection = ComputeDetection(hit, SensorTransform);
			if (PostprocessDetection(Detection))
				LidarData.WritePointSync(Detection);
			else
				PointsPerChannel[idxChannel]--;
		}
	}

	LidarData.WriteChannelCount(PointsPerChannel);
}
std::vector<std::string> ARayCastLidarWithFog::SplitString(const std::string& Input, char Delimiter) const
{
	std::vector<std::string> Tokens;
	size_t Start = 0;
	size_t End = Input.find(Delimiter);

	while (End != std::string::npos)
	{
		Tokens.push_back(Input.substr(Start, End - Start));
		Start = End + 1;
		End = Input.find(Delimiter, Start);
	}

	Tokens.push_back(Input.substr(Start));

	return Tokens;
}
void ARayCastLidarWithFog::GetStepSizeData(std::string Alpha) const
{
	std::string FilePath = GetPathSeparator();
	std::string FileName = "integral_0m_to_200m_stepsize_0.1m_tau_h_20ns_alpha_" + Alpha + ".txt";
	std::string FullPath = FilePath + "/" + FileName;

	std::ifstream InputFile(FullPath);

	if (!InputFile.is_open())
	{
		std::cout << "Can not open file: " << FullPath << std::endl;
	}

	std::string Line;
	std::vector<std::string> Temp;
	std::vector<std::string> Temp1;

	while (getline(InputFile, Line))
	{
		Temp = SplitString(Line, ':');
		Temp1 = SplitString(Temp[1], ',');
		StepSizeData[Temp[0]] = { Temp1[0], Temp1[1] };
	}

	InputFile.close();
}

std::string ARayCastLidarWithFog::GetAlphaByFogDensity(float FogDensity) const
{
	if (FogDensity > 3 && FogDensity <= 10)
	{
		return "0.01";
	}
	else if (FogDensity > 10 && FogDensity <= 25)
	{
		return "0.02";
	}
	else if (FogDensity > 25 && FogDensity <= 40)
	{
		return "0.03";
	}
	else if (FogDensity > 40 && FogDensity <= 50)
	{
		return "0.06";
	}
	else if (FogDensity > 50 && FogDensity <= 80)
	{
		return "0.1";
	}
	else if (FogDensity > 80 && FogDensity <= 90)
	{
		return "0.12";
	}
	else if (FogDensity > 90 && FogDensity <= 95)
	{
		return "0.15";
	}
	else if (FogDensity > 95)
	{
		return "0.2";
	}
	return "0.005";
}

float ARayCastLidarWithFog::GetReflectivityFromHitResult(const FHitResult& HitResult) const
{
	TWeakObjectPtr<class UPrimitiveComponent> HitComponent = HitResult.Component.Get();
	float Reflectivity = 0.1f;
	if (HitComponent != nullptr)
	{
		UMaterialInterface* Material = HitComponent->GetMaterial(HitResult.FaceIndex);
		if (Material != nullptr)
		{
			float Roughness = 0.01;
			Material->GetScalarParameterValue(TEXT("Roughness"), Roughness);
			Reflectivity = std::pow(1 - std::sqrt(1 - Roughness), 5);
		}
	}
	if (Reflectivity < 0.01f)
	{
		Reflectivity = 0.01f;
	}
	else if (Reflectivity >= 1)
	{
		Reflectivity = 0.99f;
	}
	return Reflectivity;
}
std::string ARayCastLidarWithFog::GetPathSeparator() const {
	FString CombinedPath = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("Weather/FogData"));
	return TCHAR_TO_UTF8(*CombinedPath);
}