namespace decompiler {
static const std::unordered_map<std::string, std::unordered_set<std::string>> Jak2ToggleProtos = {
  {"strip", {
    "strip-ev-base-ring.mb",
    "strip-ev-base-top.mb",
    "strip-ev-base.mb",
    "strip-ev-panel.mb",
    "strip-ev-pipe-01.mb",
    "strip-ev-pipe-02.mb",
    "strip-ev-pipe-03.mb",
    "strip-ev-tank.mb",
    "strip-ev-band.mb",
    "strip-ev-little-block.mb",

    "strip-pipe-01.mb",
    "strip-pipe-02-nut-drop.mb",
    "strip-shrub-nut-drop.mb",
    "strip-shrub-yellow-stripe.mb",
    "strip-pipe-col-disappear.mb",

    "strip-blown-up-vent-base.mb",
    "strip-blown-up-vent-pieces.mb",

    "strip-blocker-crate-01.mb",
    "strip-blocker-crate-02.mb",
    "strip-blocker-crate-03.mb",
    "strip-blocker-crate-04.mb",
    "strip-blocker-crate-05.mb",
    "strip-blocker-crate-06.mb",

    "lowres-casboss.mb"
    }
  },
  {"ruins", {
    "ruins-board-task2.mb",
    "ruins-lgcollision-task2.mb",
    "ruins-plank-task2.mb",
    "ruins-smlcollision-task2.mb",
    "ruins-support-task2.mb",

    "ruin-tower-junk.mb",

    "ruin-balcony-01-tower.mb",
    "ruin-balcony-02-tower.mb",
    "ruin-bar-01-tower.mb",
    "ruin-bar-02-tower.mb",
    "ruin-bar-03-tower.mb",
    "ruin-bridge-01-tower.mb",
    "ruin-lamp-post-01-tower.mb",
    "ruin-lamp-post-03-tower.mb",
    "ruin-lamp-post-04-tower.mb",
    "ruin-lampbase-02-tower.mb",
    "ruin-lamplite-01-tower.mb",
    "ruin-pillar-broken-01-tower.mb",
    "ruin-pillar-broken-03-tower.mb",
    "ruin-top-tower.mb",
    "ruin-tower-window-01.mb",
    "ruin-window-01-tower.mb",
    "ruins-city-corner-roof-tower.mb",
    "ruins-city-roof-01-tower.mb",
    "ruins-cracked-roof-tower.mb",
    "ruins-pipe-2m-end-tower.mb",
    "ruins-pipe-elbow-tower.mb",
    "ruins-pipe-mid-tower.mb",
    "ruins-pipe-ring-tower.mb",
    "ruins-support-01-tower.mb",
    "ruins-support-02-tower.mb",
    "swingpole-geo.mb",
    "ruin-top-brick-01.mb",
    "ruin-brick-side-01.mb"
    }
  },
  {"atoll", {
    "atoll-tank.mb",

    "lowres-casboss.mb"
    }
  },
  {"ctymarkb", {
    "city-mark-roof-before-broken.mb",

    "city-mark-roof-broken.mb"
    }
  },
  {"ctypal", {
    "ctyp-statue-wall-breakable.mb",

    "ctyp-statue-rubble-a.mb",
    "ctyp-statue-rubble-b.mb",
    "ctyp-statue-rubble-big-a.mb"
    }
  },
  {"sewer", { //Should all be one, but data instead of switch.
    "sewer-c-connect-door.mb",

    "sewer-hover-door.mb"
    }
  },
  {"sewerb", {
    "sewer-c-connect-door.mb",

    "sewer-hover-door.mb"
    }
  },
  {"sewesc", {
    "sewer-c-connect-door.mb",

    "sewer-hover-door.mb"
    }
  },
  {"sewescb", {
    "sewer-c-connect-door.mb",

    "sewer-hover-door.mb"
    }
  },
  {"ctyasha", {
    "cty-tanker-barrel.mb"
    }
  },
  {"consite", {
    "consite-barrel-broken.mb",
    "consite-cor-sheet-8x16-hi-broken.mb",
    "consite-scaffold-assmb-24m-mid-broken.mb",
    "consite-scaffold-beam-4m-broken.mb",
    "consite-scaffold-beam-8m-broken.mb",
    "consite-scaffold-i-hook-broken.mb",
    "consite-scaffold-i-span-broken.mb",
    "consite-scaffold-t-connector-broken.mb",
    "consite-scaffold-x-connector-corner-broken.mb",
    "consite-scaffold-x-connector-corner-out-broken.mb",
    "consite-plank-double-broken.mb",
    "consite-plank-single-broken.mb",
    "consite-rope-14m-broken.mb",
    "consite-rope-8m-broken.mb",
    "consite-rope-ring-broken.mb",
    "consite-scaffold-x-connector-broken.mb"
    }
  },
  {"caspad", {
    "cpad-bigtank-side.mb",
    "cpad-bigtank-top.mb",
    "cpad-bigtank-top-details.mb",
    "cpad-crane.mb",
    "cpad-crane-base.mb",
    "cpad-elev-scaffolding.mb",
    "cpad-elev-shaft-ex.mb",
    "cpad-elev-shaft-ex-detail.mb",
    "cpad-elev-shaft-roof.mb",
    "cpad-liltank-side.mb",
    "cpad-liltank-top.mb",
    "cpad-pipe-base.mb",
    "cpad-pipe-flat.mb",
    "cpad-pipe-lil-elbo.mb",
    "cpad-pipe-lil-strt.mb",
    "cpad-pipe-med-elbo.mb",
    "cpad-pipe-med-strt.mb",
    "cpad-pipe-tank-45.mb",
    "cpad-pipe-tank-strt.mb",
    "cpad-scaffold-structure.mb",
    "cpad-scaff-x-beam.mb",
    "cpad-stonework.mb",
    "cpad-top.mb",
    "cpad-tower-bottom.mb",
    "cpad-tower-centrifuse.mb",
    "cpad-tower-generator.mb",
    "cpad-tower-generator-panels.mb",
    "cpad-tower-smokestack.mb",
    "cpad-tower-supports-lower.mb",
    "cpad-tower-turbine.mb",
    "cpad-tower-walkway-lower.mb",
    "cpad-x-beam.mb"
    }
  },
  {"stadiumb", {
    "stdmb-tunnel-ramp-reverse.mb",

    "stdmb-tunnel-ramp.mb"
    }
  },
  {"hiphog", {
    "hip-paintings-bar-a.mb",
    "hip-paintings-wall-reflection-a.mb",
    "hip-paintings-wall-a.mb"

    "hip-paintings-bar-b.mb",
    "hip-paintings-wall-reflection-b.mb",
    "hip-paintings-wall-b.mb"
    }
  }
};

static const std::unordered_map<std::string, std::unordered_set<std::string>> Jak3ToggleProtos = {
  {"foresta", { //Should all be one, but data instead of switch.
    "gun-base-cylinder.mb", //Always #f? Gonna have to take a look at this model.
    "gun-base-tubes.mb",

    "destroyed-statue-base.mb",
    "destroyed-statue-chunk-a.mb",
    "destroyed-statue-chunk-b.mb",
    "destroyed-statue-eyelid.mb",
    "destroyed-statue-rubble-terrain.mb",
    "fora-grounder-destroyed-statue-lil-rocks.mb",
    "fora-shrub-destroyed-statue-pebbles.mb",
    "neo-spawner-root-a.mb",
    "neo-spawner-root-b.mb",

    "green-eco-vent-a.mb"
    }
  },
  {"forestb", {
    "gun-base-cylinder.mb", //Always #f? Gonna have to take a look at this model.
    "gun-base-tubes.mb",

    "destroyed-statue-base.mb",
    "destroyed-statue-chunk-a.mb",
    "destroyed-statue-chunk-b.mb",
    "destroyed-statue-eyelid.mb",
    "destroyed-statue-rubble-terrain.mb",
    "fora-grounder-destroyed-statue-lil-rocks.mb",
    "fora-shrub-destroyed-statue-pebbles.mb",
    "neo-spawner-root-a.mb",
    "neo-spawner-root-b.mb",

    "green-eco-vent-a.mb"
    }
  },
  {"mhcityb", {  //Should all be one, but data instead of switch.
    "mhcity-de-tower-grind-strand-long-nocol01.mb",
    "mhcity-ground-lower-wall-gapfiller-strand-nub-01.mb",
    "mhcity-ground-lower-wall-veins-small-02.mb",
    "mhcity-wall-vine-thin-c-destrand-01.mb",
    "mhcity-wall-vine-thin-s-destrand-01.mb"
    }
  },
  {"mhcitya", {
    "mhcity-de-tower-grind-strand-long-nocol01.mb",
    "mhcity-ground-lower-wall-gapfiller-strand-nub-01.mb",
    "mhcity-ground-lower-wall-veins-small-02.mb",
    "mhcity-wall-vine-thin-c-destrand-01.mb",
    "mhcity-wall-vine-thin-s-destrand-01.mb"
    }
  },
  {"factoryb", {
    "facb-gun-tower-base-01.mb",

    "facb-gun-tower-base-02.mb",

    "facb-gun-tower-base-03.mb",

    "facb-gun-tower-base-04.mb"
    }
  },
  {"precurd", {
    "precur-road-bridge01.mb",
    "precur-road-bridge02.mb",
    "precur-road-bridge03.mb",
    "precur-road-bridge04.mb",
    "precur-road-bridge05.mb"
    }
  },
  {"desertg", {
    "des-egg.mb"
    }
  },
  {"wasstada", {
    "wstd-table.mb"
    }
  },
  {"desertg", {
    "des-egg.mb"
    }
  },
  {"templea", { //Should all be one, but data instead of switch.
    "tpl-hide-debris-blocka.mb",
    "tpl-hide-debris-blockb.mb",
    "tpl-hide-debris-plank-8m.mb",
    "tpl-hide-debris-rock1.mb",
    "tpl-hide-sunken-brick-01.mb",
    "tpl-hide-collision-01.mb",

    "tpl-hall-alter-tunnel-section-jnt-hide.mb,"
    "tpl-hall-alter-tunnel-section-hide.mb"
    }
  },
  {"templed", {
    "tpl-hide-debris-blocka.mb",
    "tpl-hide-debris-blockb.mb",
    "tpl-hide-debris-plank-8m.mb",
    "tpl-hide-debris-rock1.mb",
    "tpl-hide-sunken-brick-01.mb",
    "tpl-hide-collision-01.mb",

    "tpl-hall-alter-tunnel-section-jnt-hide.mb,"
    "tpl-hall-alter-tunnel-section-hide.mb"
    }
  },
  {"ctywide", {
    "palcab-lowres-mhcity-tower-shell.mb",
    "palcab-lowres-mhcity-tower-shelleye-01.mb",
    "palcab-lowres-mhcity-tower-cap.mb",
    "palcab-lowres-mhcity-tower-capdoor-01.mb",

    "palcab-lowres-mhcity-tower-shell-blasted-01.mb",
    "palcab-lowres-mhcity-tower-shell-blasted-02.mb"
    }
  },
  {"ctygenb", {
    "city-cable-grindable-turnoff-01.mb",
    "city-cable-grindable-collision-01.mb",
    "city-cable-grindable-collision.mb"
    }
  },
  {"atoll", {

    }
  },
  {"ctymarkb", {

    }
  },
  {"ctypal", {

    }
  },
  {"ctyasha", {

    }
  },
  {"stadiumb", {

    }
  },
  {"hiphog", {

    }
  },
};

static bool isProtoToggleable(std::string levelName, std::string protoName, GameVersion version) {
  std::list<std::pair<const std::string, std::unordered_set<std::string>>, std::allocator<std::pair<const std::string, std::unordered_set<std::string>>>>::const_iterator level;
  switch (version) {
    case GameVersion::Jak1:
      return false;
    case GameVersion::Jak2:
      level = Jak2ToggleProtos.find(levelName);
      if (level == Jak2ToggleProtos.end())
        return false;
      return level->second.contains(protoName);
    case GameVersion::Jak3:
      level = Jak3ToggleProtos.find(levelName);
      if (level == Jak3ToggleProtos.end())
        return false;
      return level->second.contains(protoName);
    case GameVersion::JakX:
      return false; //No idea, I'd assume not? Gotta wait for decomp to be 100% certain,
                    //might be used for track wall toggles?
    default:
      ASSERT_NOT_REACHED();
  }
  return false;
}
}
