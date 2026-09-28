////////////////////////////////////////////////////////////////////////
// Class:       FilterNoSimEnergyDeposits
// Module Type: filter
// File:        FilterNoSimEnergyDeposits_module.cc
//
// Module for rejecting events in which no appreciable energy was
// deposited in the detector, using the sim::SimEnergyDeposit data
// product as input.
//
// Generators that simulate particles originating outside the detector
// -- cosmic ray muons in particular -- produce a large fraction of
// events in which no primary ever reaches an active volume. Those
// events are still propagated through the full detector simulation and
// reconstruction chain, where the dominant costs (electronics noise on
// every readout channel, trigger primitive finding) are incurred per
// event rather than per deposit, and so are paid in full for an empty
// event. Placing this filter after the ionisation and scintillation
// step and before the detector simulation lets such events be dropped
// before that cost is incurred.
//
// An event is accepted when the deposits summed over all of the
// configured input collections satisfy both thresholds: at least
// MinNumberOfDeposits deposits, and at least MinTotalEnergy of
// deposited energy. Either threshold may be disabled by setting it to
// zero. The input is a sequence of tags rather than a single tag
// because a LArG4 stage may split its deposits across many instance
// names, one per volume.
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Core/SharedFilter.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/types/Atom.h"
#include "fhiclcpp/types/Comment.h"
#include "fhiclcpp/types/Name.h"
#include "fhiclcpp/types/Sequence.h"
#include "messagefacility/MessageLogger/MessageLogger.h"

#include "lardataobj/Simulation/SimEnergyDeposit.h"

#include <vector>

namespace simfilter {
  class FilterNoSimEnergyDeposits;
}

class simfilter::FilterNoSimEnergyDeposits : public art::SharedFilter {
public:
  struct Config {

    fhicl::Sequence<art::InputTag> SimEnergyDepositLabels{
      fhicl::Name{"SimEnergyDepositLabels"},
      fhicl::Comment{"labels of the sim::SimEnergyDeposit collections to examine"}};

    fhicl::Atom<unsigned int> MinNumberOfDeposits{
      fhicl::Name{"MinNumberOfDeposits"},
      fhicl::Comment{"minimum number of deposits, summed over all collections,"
                     " for an event to be accepted (0 disables this requirement)"},
      1 // default
    };

    fhicl::Atom<double> MinTotalEnergy{
      fhicl::Name{"MinTotalEnergy"},
      fhicl::Comment{"minimum deposited energy [MeV], summed over all collections,"
                     " for an event to be accepted (0 disables this requirement)"},
      0.0 // default
    };

    fhicl::Atom<bool> ThrowOnMissingProduct{
      fhicl::Name{"ThrowOnMissingProduct"},
      fhicl::Comment{"whether a collection absent from the event is an error;"
                     " when false, such a collection contributes nothing"},
      true // default
    };
  };

  using Parameters = art::SharedFilter::Table<Config>;

  explicit FilterNoSimEnergyDeposits(Parameters const& config, art::ProcessingFrame const&);

  // Plugins should not be copied or assigned.
  FilterNoSimEnergyDeposits(FilterNoSimEnergyDeposits const&) = delete;
  FilterNoSimEnergyDeposits(FilterNoSimEnergyDeposits&&) = delete;
  FilterNoSimEnergyDeposits& operator=(FilterNoSimEnergyDeposits const&) = delete;
  FilterNoSimEnergyDeposits& operator=(FilterNoSimEnergyDeposits&&) = delete;

private:
  bool filter(art::Event& e, art::ProcessingFrame const&) override;

  std::vector<art::InputTag> const fSimEnergyDepositLabels; //!< Collections to examine.
  unsigned int const fMinNumberOfDeposits;                  //!< Minimum deposits to accept.
  double const fMinTotalEnergy;                             //!< Minimum energy [MeV] to accept.
  bool const fThrowOnMissingProduct;                        //!< Whether a missing product errors.
};

simfilter::FilterNoSimEnergyDeposits::FilterNoSimEnergyDeposits(Parameters const& config,
                                                                art::ProcessingFrame const&)
  : SharedFilter{config}
  , fSimEnergyDepositLabels{config().SimEnergyDepositLabels()}
  , fMinNumberOfDeposits{config().MinNumberOfDeposits()}
  , fMinTotalEnergy{config().MinTotalEnergy()}
  , fThrowOnMissingProduct{config().ThrowOnMissingProduct()}
{
  // This module only reads from the event and keeps no mutable state,
  // so event-level calls may proceed asynchronously.
  async<art::InEvent>();
}

bool simfilter::FilterNoSimEnergyDeposits::filter(art::Event& e, art::ProcessingFrame const&)
{
  std::size_t nDeposits{0};
  double totalEnergy{0.};

  for (auto const& label : fSimEnergyDepositLabels) {

    std::vector<sim::SimEnergyDeposit> const* deposits{nullptr};

    if (fThrowOnMissingProduct) {
      deposits = e.getValidHandle<std::vector<sim::SimEnergyDeposit>>(label).product();
    }
    else {
      art::Handle<std::vector<sim::SimEnergyDeposit>> handle;
      if (not e.getByLabel(label, handle)) {
        mf::LogDebug("FilterNoSimEnergyDeposits")
          << "No sim::SimEnergyDeposit collection '" << label.encode() << "' in this event;"
          << " treating it as empty.";
        continue;
      }
      deposits = handle.product();
    }

    nDeposits += deposits->size();
    for (auto const& edep : *deposits) {
      totalEnergy += edep.Energy();
    }

    mf::LogDebug("FilterNoSimEnergyDeposits")
      << "Collection '" << label.encode() << "': " << deposits->size() << " deposits;"
      << " running totals " << nDeposits << " deposits, " << totalEnergy << " MeV.";

    // Both thresholds are monotonic in the running totals, so once they
    // are met the remaining collections cannot change the decision.
    if (nDeposits >= fMinNumberOfDeposits and totalEnergy >= fMinTotalEnergy) {
      mf::LogInfo("FilterNoSimEnergyDeposits")
        << "Accepting event: " << nDeposits << " deposits totalling " << totalEnergy << " MeV.";
      return true;
    }
  }

  mf::LogInfo("FilterNoSimEnergyDeposits")
    << "Rejecting event: " << nDeposits << " deposits totalling " << totalEnergy << " MeV,"
    << " below the required " << fMinNumberOfDeposits << " deposits and " << fMinTotalEnergy
    << " MeV.";

  return false;
}

DEFINE_ART_MODULE(simfilter::FilterNoSimEnergyDeposits)
