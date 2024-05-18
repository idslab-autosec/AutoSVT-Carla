// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include <vector>
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

ARayCastLidarWithFog::FDetection ARayCastLidarWithFog::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf)
{
	FDetection Detection;
	Detection.object_tag = 0;
	const FVector HitPoint = HitInfo.ImpactPoint;
	Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);
	const float Distance = Detection.point.Length();

	// const float AttenAtm = Description.AtmospAttenRate;
	// const float AbsAtm = exp(-AttenAtm * Distance);
	// float OriginalIntensity = static_cast<uint32_t>(AbsAtm * 255);
	

	// Get reflectivity
	float Gamma = 0.0000021f;
	// float Gamma = 0.21f;
	const FActorRegistry& Registry = GetEpisode().GetActorRegistry();
	Detection.object_tag = static_cast<uint32_t>(HitInfo.Component->CustomDepthStencilValue);
	Gamma = GetReflectivity(Detection.object_tag) / std::pow(10, 5);

	float Beta0 = Gamma / M_PI;
	float OriginalIntensity = 5e9; // C_A P_0

	FWeatherParameters weather = GetEpisode().GetWeather()->GetCurrentWeather();
	float FogDensity = weather.FogDensity;
	
	if (FogDensity < 0)
	{
		FogDensity = 0;
	}
	else if (FogDensity > 100)
	{
		FogDensity = 100;
	}

	// Fog density changed
	if (CurrentFogDensity != FogDensity || isinf(Alpha))
	{
		CurrentFogDensity = FogDensity;
		GetStepSizeData(CurrentFogDensity);
		Mor = CalculateMOR(FogDensity);
		Alpha = std::log(20) / Mor;
		Beta = 0.046f / Mor;
	}
	if (FogDensity == 0)
	{
		// Suppose MOR = 10km in clear weather
		Alpha = 0.000229f; 
		Beta = 0.0000046f;
	}

	if (FogDensity > 0)
	{
		// Hard
		float Hard = OriginalIntensity * Beta0 / std::pow(Distance, 2.0) * exp(-2 * Alpha * Distance);
		Hard = std::min(Hard, 255.0f);
		// Soft
		char buffer[20];
		std::snprintf(buffer, sizeof(buffer), "%.1f", std::min(Distance, 200.0f));
		std::string Key(buffer);

		// R_tmp, i_soft
		const std::vector<std::string>& Data = StepSizeData.at(Key);
		float StepDataDistance = std::stof(Data[0]);
		double Soft = std::stod(Data[1]); // max(Simpson), i.e., i_{tmp}
		Soft = OriginalIntensity * Beta * Soft;

		// i_soft
		if (Soft > 255)
		{
			Soft = 255;
		}
		Detection.intensity = Hard;

		if (Soft > Hard)
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
			Detection.intensity = Soft;
			Detection.object_tag = 29;
		}
		// UE_LOG(LogTemp, Warning, TEXT("FogDensity = %f, Mor = %f, Alpha = %f"), FogDensity, Mor, Alpha);
		// UE_LOG(LogTemp, Warning, TEXT("soft_i = %f, hard_i = %f, object_tag = %d"), Soft, Hard, Detection.object_tag); 
	}
	else
	{	
		// clear weather
		Detection.intensity = OriginalIntensity * Beta0 / std::pow(Distance, 2.0) * exp(-2 * Alpha * Distance);
	}
	Detection.point.y *= -1; // For Carla-Apollo-Bridge
	// UE_LOG(LogTemp, Warning, TEXT("object_tag = %d"), Detection.object_tag); 
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
void ARayCastLidarWithFog::GetStepSizeData(float FogDensity) const
{
	std::string Alpha = GetAlphaInFileName(FogDensity);
	std::string FilePath = GetPathSeparator();
	std::string FileName = "integral_0m_to_200m_stepsize_0.1m_tau_h_20ns_alpha_" + Alpha + ".txt";
	std::string FullPath = FilePath + "/" + FileName;
	std::ifstream InputFile(FullPath);

	std::string Line;
	std::vector<std::string> Temp;
	std::vector<std::string> Data;

	while (getline(InputFile, Line))
	{
		Temp = SplitString(Line, ':');
		Data = SplitString(Temp[1], ',');
		StepSizeData[Temp[0]] = {Data[0], Data[1]};
	}

	InputFile.close();
}

std::string ARayCastLidarWithFog::GetAlphaInFileName(float FogDensity) const
{
	if (FogDensity <= 10) {
		return "0.005";
	}
	else if (FogDensity <= 22) {
		return "0.01";
	}
	else if (FogDensity <= 30.5) {
		return "0.015";
	}
	else if (FogDensity <= 36.5) {
		return "0.02";
	}
	else if (FogDensity <= 59) {
		return "0.03";
	}
	else if (FogDensity <= 71) {
		return "0.06";
	}
	else if (FogDensity <= 84) {
		return "0.1";
	}
	else if (FogDensity <= 92.5) {
		return "0.15";
	}
	else if (FogDensity <= 100) {
		return "0.2";
	}
	else {
		return "0.005";
	}
}


float ARayCastLidarWithFog::PiecewiseLinearRegression(const std::vector<FogDensityDataPoint>& data, float x) const
{
    for (size_t i = 1; i < data.size(); ++i) {
        if (x >= data[i - 1].x && x < data[i].x) {
            float slope = (data[i].y - data[i - 1].y) / (data[i].x - data[i - 1].x);
            float y_intercept = data[i - 1].y - slope * data[i - 1].x;
            return slope * x + y_intercept;
        }
    }
    return 0.0;
}

float ARayCastLidarWithFog::CalculateMOR(const float FogDensity) const
{
	std::vector<FogDensityDataPoint> FogDensityData = {
        {2, 600},
        {10, 300},
        {15, 200},
        {25, 150},
        {40, 100},
        {50, 50},
        {80, 30},
        {90, 25},
        {95, 20},
        {100, 15}
    };
	float NewMOR = PiecewiseLinearRegression(FogDensityData, FogDensity);
	if (NewMOR <= 0) {
		NewMOR = 10000;
	}
	return NewMOR;
}

// TSet<FString> UniqueLabels; // debug

float ARayCastLidarWithFog::GetReflectivity(uint32_t ObjectTag) const
{
	// ObjectTag: https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera
	if (ObjectTag == 0) {
		return 0.21f;
	}
	else if (ObjectTag == 1 || ObjectTag == 2) {
		return 0.2f;
	}
	else if (ObjectTag == 3) {
		return 0.32f;
	}
	else if (ObjectTag == 4) {
		return 0.21f;
	}
	else if (ObjectTag == 5) {
		return 0.24f;
	}
	else if (ObjectTag == 6) {
		return 0.08f;
	}
	else if (ObjectTag == 8) {
		return 0.37f;
	}
	else if (ObjectTag == 9) {
		return 0.17f;
	}
	else if (ObjectTag == 10) {
		return 0.19f;
	}
	else if (ObjectTag == 11) {
		return 0.0f;
	}
	else if (ObjectTag == 12 || ObjectTag == 13) {
		return 0.09f;
	}
	else if (ObjectTag == 14 || ObjectTag == 15 || ObjectTag == 16 || ObjectTag == 17) {
		return 0.06f;
	}
	else if (ObjectTag == 18) {
		return 0.08f;
	}
	else if (ObjectTag == 19) {
		return 0.13f;
	}
	else if (ObjectTag == 23) {
		return 0.06f;
	}
	else {
		return 0.21;
	}
}

float ARayCastLidarWithFog::LookupReflectivityTable(FString ActorLabel) const 
{
	if ( // terrain
		ActorLabel.Contains(TEXT("grass")) || 
		ActorLabel.Contains(TEXT("terrain"))
	)
	{
		return 0.19f;
	}

	else if ( // road, sidewalk
		ActorLabel.Contains(TEXT("block")) || 
		ActorLabel.Contains(TEXT("Road_"), ESearchCase::CaseSensitive) || 
		ActorLabel.Contains(TEXT("line"))
	)
	{
		return 0.2f;
	}

	else if ( // building
		ActorLabel.Contains(TEXT("apartment")) || 
		ActorLabel.Contains(TEXT("house")) || 
		ActorLabel.Contains(TEXT("office")) || 
		ActorLabel.Contains(TEXT("staticmesh")) || 
		ActorLabel.Contains(TEXT("concrete")) || 
		ActorLabel.Contains(TEXT("skyscraper")) || 
		ActorLabel.Contains(TEXT("mall")) ||
		ActorLabel.Contains(TEXT("shop_"))
	)
	{
		return 0.32f;
	}

	else if ( // parking
		ActorLabel.Contains(TEXT("parking"))
	)
	{
		return 0.1f;
	}

	else if ( // other-structure
		ActorLabel.Contains(TEXT("SM_"), ESearchCase::CaseSensitive) || 
		ActorLabel.Contains(TEXT("wall")) || 
		ActorLabel.Contains(TEXT("sculpture")) || 
		ActorLabel.Contains(TEXT("tunelentrance")) || 
		ActorLabel.Contains(TEXT("_bin")) || 
		ActorLabel.Contains(TEXT("barrel")) || 
		ActorLabel.Contains(TEXT("lamppost")) || 
		ActorLabel.Contains(TEXT("repspline"))
	)
	{
		return 0.21f;
	}

	else if ( // fence
		ActorLabel.Contains(TEXT("fence"))
	)
	{
		return 0.24f;
	}
	
	else if ( // pole
		ActorLabel.Contains(TEXT("light"))
	)
	{
		return 0.08f;
	}

	else if ( // traffic-sign
		ActorLabel.Contains(TEXT("speedlimit"))
	)
	{
		return 0.37f;
	}

	if ( // person
		ActorLabel.Contains(TEXT("BP_Walker"), ESearchCase::CaseSensitive) || 
		ActorLabel.Contains(TEXT("bush"))
	)
	{
		return 0.09f;	
	}

	if ( // car, truck
		ActorLabel.Contains(TEXT("BP_"), ESearchCase::CaseSensitive) || 
		ActorLabel.Contains(TEXT("Vh_"), ESearchCase::CaseSensitive)
	)
	{
		return 0.06f;	
	}

	else if ( // vegetation
		ActorLabel.Contains(TEXT("leaf")) || 
		ActorLabel.Contains(TEXT("pine")) || 
		ActorLabel.Contains(TEXT("foliage")) || 
		ActorLabel.Contains(TEXT("maple")) || 
		ActorLabel.Contains(TEXT("palmera")) || 
		ActorLabel.Contains(TEXT("platanus")) || 
		ActorLabel.Contains(TEXT("sassafras")) || 
		ActorLabel.Contains(TEXT("Veg_"), ESearchCase::CaseSensitive) ||
		ActorLabel.Contains(TEXT("bush")) || 
		ActorLabel.Contains(TEXT("leave"))
	)
	{
		return 0.17f;	
	}

	else
	{
		return 0.21f;
	}
}

std::string ARayCastLidarWithFog::GetPathSeparator() const 
{
	FString CombinedPath = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("Weather/FogData"));
	return TCHAR_TO_UTF8(*CombinedPath);
}