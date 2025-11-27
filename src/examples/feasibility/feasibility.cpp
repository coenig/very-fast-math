// #vfm-insert-includes

// #vfm-begin
// #vfm-ltl-spec[[ true ]]
// #vfm-insert-associativity-pretext
si_scalar_f32_t Object2EgoInteraction__calcFeasibility(
   const si_metre_f32_t                   dSafeMin,
   const si_metre_f32_t                   dDeltaSafe,
   const si_metre_per_square_second_f32_t aMax,
   const si_metre_per_square_second_f32_t aDelta,
   const si_metre_per_square_second_f32_t aObjectReaction,
   const si_second_f32_t                  timeToDecelerate,
   const si_metre_per_second_f32_t        upperVelocityLimitScalingTransitionTowardsRecedingObjects,
   const si_metre_f32_t                   dxObj,
   const si_metre_per_second_f32_t        vxRel)
{
   // #vfm-gencode-begin[[ condition=false ]]
   
   // #vfm-cutout-begin
   VFC_REQUIRE(dSafeMin >= si_metre_f32_t{0.0F});
   VFC_REQUIRE(dDeltaSafe >= si_metre_f32_t{0.0F});
   VFC_REQUIRE(aMax <= si_metre_per_square_second_f32_t{0.0F});
   VFC_REQUIRE(aDelta >= si_metre_per_square_second_f32_t{0.0F});
   VFC_REQUIRE(dxObj >= si_metre_f32_t{0.0F});
   // #vfm-cutout-end

   si_scalar_f32_t feasibility{0.0F};

   // kappa is used to adjust the discontinuous switches of the acceleration assumed for the feasibility
   // calculation towards a more realistic continuous acceleration
   constexpr vfc::float32_t kappa = 2.0F;

   const bool isOpeningGapScenario = vxRel > si_metre_per_second_f32_t{0.0F};
   // simplified feasibility equation for "opening gap" scenario
   // assuming that the object is moving away from the ego vehicle
   // aObjectReaction is approximated deceleration interaction object can react on the ego vehicle
   if (isOpeningGapScenario)
   {
      const auto reactionTermScaling = Lib::interpolateLinearlyAndClamp(
         vxRel,
         si_metre_per_second_f32_t{0.0F},
         si_scalar_f32_t{0.0F},
         vfc ::max(si_metre_per_second_f32_t{0.F}, upperVelocityLimitScalingTransitionTowardsRecedingObjects),
         si_scalar_f32_t{1.0F},
         Lib::DiscontinuityBehavior::midPoint);

      const si_metre_f32_t distanceToDecelerate =
         si_scalar_f32_t{0.5F} * aObjectReaction * timeToDecelerate * timeToDecelerate;

      const auto scaledDistanceToDecelerate = reactionTermScaling * distanceToDecelerate;

      const auto lower = dSafeMin;
      const auto upper = vfc::max(lower, dSafeMin + dDeltaSafe - scaledDistanceToDecelerate);
      feasibility      = Lib::interpolateLinearlyAndClamp(
         dxObj,
         lower,
         si_scalar_f32_t{0.0F},
         upper,
         si_scalar_f32_t{1.0F},
         Lib::DiscontinuityBehavior::midPoint);
   }
   // Prevent division by zero of the feasibility equation. Transform formula in these cases
   else if (
      vfc::isZero(aDelta) || vfc::isZero(aDelta * dDeltaSafe)
      || vfc::isZero(aDelta * (dxObj - dSafeMin)))  // aDelta == 0
   {
      if (vfc::notZero(aMax))
      {
         // simplified feasibility equation for this condition
         // feasibility = (dxObj - dSafeMin + requiredDecelerationDistance) / dDeltaSafe
         // with requiredDecelerationDistance = (vxRel * vxRel / (kappa * aMax))
         const auto requiredDecelerationDistance = vxRel * vxRel / (kappa * aMax);
         const auto lower                        = dSafeMin - requiredDecelerationDistance;
         const auto upper                        = dSafeMin + dDeltaSafe - requiredDecelerationDistance;
         feasibility                             = Lib::interpolateLinearlyAndClamp(
            dxObj,
            lower,
            si_scalar_f32_t{0.0F},
            upper,
            si_scalar_f32_t{1.0F},
            Lib::DiscontinuityBehavior::midPoint);
      }
   }
   else if (vfc::isZero(dDeltaSafe))  // dDeltaSafe == 0 AND aDelta != 0
   {
      feasibility = -(aMax / aDelta) - (vxRel * vxRel / (kappa * aDelta * (dxObj - dSafeMin)));
   }
   else  // dDeltaSafe != 0 AND aDelta != 0
   {
      // Use feasibility equation F = signAlpha * alpha - sqrt(alpha^2 + beta + gamma)
      // Buffer variables in feasibility equation : F = a - b - sqrt((a + b)^2 + gamma)
      const si_scalar_f32_t a        = (dxObj - dSafeMin) / (2.0F * dDeltaSafe);
      const si_scalar_f32_t b        = aMax / (2.0F * aDelta);
      const si_scalar_f32_t alpha    = a + b;
      const si_scalar_f32_t gamma    = vxRel * vxRel / (kappa * aDelta * dDeltaSafe);
      const si_scalar_f32_t radicand = alpha * alpha + gamma;

      // The radicand is guaranteed to be positive if dDeltaSafe and aDelta are both positive.
      VFC_ASSERT(radicand >= si_scalar_f32_t{0.0F});
      feasibility = a - b - vfc::sqrt(radicand);
   }

   return feasibility;   // #vfm-gencode-end
}
// #vfm-end
