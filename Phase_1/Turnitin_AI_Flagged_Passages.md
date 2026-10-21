# Turnitin AI-flagged passages (submission trn:oid:::1:3686020387, 43%)

Each block is text Turnitin highlighted. Rewrite these in your own words.
Page = printed page number in the report.
Total flagged: about 4863 words.


## Front matter (Acknowledgements / Abstract / Contents / Lists)

**PDF p3** (23 words)

> Our sincere thanks to Dr. T.T. MIRNALINEE, Professor and Head of the Department of Computer Science and Engineering, for her encouragement and support.

**PDF p4** (80 words)

> Every Pareto point is scored by these weights via a utility function, and the argmax is selected as the mission route. The principal contribution is a formula-driven mapping from metadata to routing weights, linking mission context directly to Pareto selection without expert intervention. Route choice is set by payload properties rather than context-blind diversity criteria. The Pareto front is stored and rebuilt only when the graph changes, its validity window expires or the weather drifts, serving all missions between updates.

**PDF p5** (44 words)

> iii viii ix x Making a front and picking from it . What deployed services tell us . How weather enters the problem . What goes in, what comes out . POPSO with a Pareto-entropy archive . CDNSGA-II over an urban octree . iv

**PDF p6** (75 words)

> Where the map and weather come from . The lattice and which nodes stay . Linking nodes, then dropping edges . Risk terms that ignore the weather . Weather risk and the go/no-go gate . Labels and when one beats another . A heuristic that never overestimates . Capping the search and the front . Why we chose exact search . Keeping the front, and when to rebuild . 3.5.2 From tags to weights .

**PDF p7** (47 words)

> How we score a front . Sweeping all 225 tag sets . Benchmarking as a loop . 3.7.1 Writing the mission file . 3.8.1 Bundle, clearance and altitude frame . 4.4.1 Tier 1 versus the baseline planners . 4.4.2 What the wind does to the front .

**PDF p8** (19 words)

> Edge costs by heading . Taking out one layer at a time . Set against the reviewed planners .

**PDF p9** (154 words)

> 1.1 What the system takes in, gives back and is judged on . Reviewed papers and where each falls short for us . Each gap and the objectives aimed at closing it . Risk terms, each scaled to [0,1], and the weight on each . Which Tier 1 objective each payload tag feeds . Weights the formula gives the four reference presets . Test graphs and scenarios, with their sizes . Where each layer of the real-region lattice comes from . Phase I experiments run up to now . First benchmark pass on S1, S2 and S3 (we want HV high, and IGD and ∆low) . Edge cost by heading on a flat lattice, 25 km/h wind from 90◦. One layer removed per run (6 km lattice, fixtures, 30 km/h SW wind) 52 Routes Tier 2 picked on the S2 (16-route) and S3 (115-route) fronts 53 Our planner beside the four reviewed planners .


## Chapter 1 Introduction

**p1** (24 words)

> Re-tuning and re-running the search between flights takes time and someone who knows the planner well, and during an emergency there is usually neither.

**p2** (78 words)

> Our Payload-Aware Two-Tier UAV Route Planning system takes that step away. Tier 1 builds the Pareto front for a delivery region once, using an exact multi-objective graph search over a three-dimensional airspace lattice in which edge costs depend on terrain, buildings, restricted zones and the way the wind blows. It is then exported as an ArduPilot mission and flown in a Gazebo and ArduPilot software-in-the-loop (SITL) simulation. We would take a longer route if its air is calmer.

**p4** (210 words)

> Exact graph search, NAMOA* [11] for one, keeps every non-dominated partial path at each node. Population methods such as NSGA-II [6] and MOPSO [5] cope better with big spaces but are random. Both have a large literature. Picking one route from it gets far less attention: the usual options are a crowding-distance rule [6, 18], an entropy criterion followed by a hand-set weighted sum [4], or a person. None of them looks at what the drone is carrying. Where payload shows up, it is a continuous mass in the energy model [21]; that alters the edge costs but not the choice among trade-offs. We found nobody using payload metadata to select a point from a front computed in advance. 1.2.2 What deployed services tell us Deployed services tell us where payload matters, though the public record is thin. Swiss Post used Matternet quadrotors to carry laboratory samples between hospitals and stopped after two incidents in 2019. The first, in January, was a short circuit that cut GPS power and forced an emergency landing on Lake Z¨urich. In May a parachute tether snagged on the airframe and the 22-pound drone fell into woods near a group of kindergarten children [2]. Both were equipment failures, so they say nothing about route choice.

**p6** (35 words)

> The front must be reused across missions between refreshes instead of being rebuilt each time. Last, the step from (F,W,U,Plvl) to p∗has to be a written formula; expert rules and lookup tables do not count.

**p7** (112 words)

> 1.3.2 How weather enters the problem Weather changes the edge costs. Let ω be a snapshot of wind speed and direction, gust, rain and visibility. Distance is fixed, while energy and risk move with the angle at which the wind meets each edge. Selection works as before. The catch is that a stored front only holds while ω stays near the snapshot behind it, so we refresh it on a timer and whenever the weather drifts. If ω passes the go/no-go limits, nothing is selected and the mission waits. 1.3.3 What goes in, what comes out Table 1.1 sets out the inputs, the outputs and the test each part has to pass.

**p8** (195 words)

> This report covers Phase I, which delivers the complete software system and its simulation. In Phase II the same system goes onto a Pixhawk-equipped F450 quadcopter with a Raspberry Pi 5 companion computer and is tested in outdoor flights. A Tier 1 front is trusted only for a bounded window. Once the window runs out, or the weather moves past set thresholds, it is rebuilt; until then Tier 2 alone serves every mission. Multi-UAV coordination, dodging moving obstacles in flight and real-time replanning are outside our scope. There are six chapters, then the references and three appendices. Chapter 1 gives the motivation, background and problem. Chapter 2 reviews the nearest papers, what they leave out, and the objectives we took from that. Chapter 3 explains the architecture, its seven modules and the algorithms inside them. Chapter 4 covers datasets, hardware and software, the experiments run so far and their results. Chapter 5 looks at social impact and sustainability, and Chapter 6 closes with the plan for the rest of Phase I and for Phase II. The appendices, after the references, list the main configuration parameters, the automated tests and how the code is laid out.


## Chapter 2 Literature Survey

**p9** (172 words)

> Work on multi-objective UAV path planning comes from two sides, graph search and evolutionary optimisation [7]. We read it asking one thing. Once a planner holds a set of good routes, how does it choose the one to fly, and does the payload get a say? Below we cover the papers nearest that question, the algorithms we build on or compete with, and the few studies of what drone transport does to medical payloads. The chapter ends with the gaps we found and the objectives we set from them. 2.1.1 POPSO with a Pareto-entropy archive Chokkalingam and Ganesan [4] propose POPSO, a Pareto-optimal particle swarm optimiser for UAV paths in changing surroundings. It has four parts. Grid-based and random particles are mixed at the start, so the swarm begins with varied paths. Path length, energy, an inverse safety term, a penalty for moving obstacles and a weather factor make up the cost. Inertia weight and learning coefficients shift during the run, pushing exploration when progress stalls and exploitation when it does not.

**p12** (9 words)

> NAMOA* is the basis of our Tier 1 engine.

**p13** (20 words)

> Theobald et al. [20] flew monoclonal antibody infusions on sixty flights with vibration between 1.5 and 3 g and found

**p14** (55 words)

> no change in particle size or aggregation. So the payloads that were tested come through normal flights unharmed. We read this as a reason to treat fragility as an operator’s precaution in the planner, not as a proven damage model. 2.1.9 The reviewed work side by side The papers above are collected in Table 2.1.

**p15** (39 words)

> Front diversity is the main design concern in two of the planners we reviewed, yet only one of them uses it to select a route: Ren et al. [18] take a point straight off the front by crowding distance.


## Chapter 3 Proposed Methodology

**p19** (54 words)

> Figure 3.1 gives an overview of the workflow, from environment data to the flown mission, with baseline planners and an evaluation module scoring each stage. FIGURE 3.1: How the work flows: Tier 1 builds the Pareto front, Tier 2 picks a route from it by payload, and the mission is exported, flown and evaluated

**p20** (17 words)

> Module 1 turns these layers into a directed airspace graph whose edges carry a three-objective cost vector.

**p21** (92 words)

> For each module, the sections below give its purpose, inputs, processing and outputs. Module 1 produces the delivery airspace graph G = (V,E) from three inputs: the region (centre, box size and lattice spacing), the environment layers covered next, and a weather snapshot ω. What comes out is a directed NetworkX [8] graph. Each node knows its position and altitude, and each edge carries its static features, a per- component risk breakdown and the cost vector [d,ε,ρ]. Two builders share one cost pipeline. One lays a lattice over a real region (below);

**p22** (244 words)

> Algorithm 1 sums up the lattice builder. 3.2.1 Where the map and weather come from Weather comes from the Open-Meteo forecast service [15]: 10 m wind, gust, rain, visibility and the WMO code, plus the hourly 80 m wind speed and direction. With the network down, the planner uses its last cached snapshot. With no cache either, it assumes calm air instead of guessing, which also keeps runs repeatable. Terrain comes from whichever elevation provider answers first, tried in the order local GeoTIFF, OpenTopography Copernicus 30 m, Open-Meteo elevation and, as a last resort, flat ground; every answer is cached on disk. OpenStreetMap buildings, restricted zones, water and land use arrive through a multi-endpoint Overpass client that retries on failure. We store them as GeoPackage files [3] and index them in a local metric frame with Shapely STRtree structures. The index is what keeps height, distance, containment and segment-intersection queries fast. 3.2.2 The lattice and which nodes stay Drones do not follow roads, so we model the airspace as a regular lattice rather than a road network. The region is covered by an n×n lattice in a local equirectangular frame centred on the region, with n = round(box/spacing). The default 6 km box at 200 m spacing yields 900 candidates, but a candidate becomes a node only if it passes a selection rule. The rule looks at four things: DEM ground elevation, the tallest building in the cell, the legal ceiling, and restricted airspace.

**p23** (152 words)

> with cruise height Hcruise = 60 m and clearance C = 15 m. A building’s height is read from its height tag, else from building:levels times 3 m, else from a default for its type. Every surviving node stores its position in both frames, its ground elevation, the building height, density and mean height of its cell, its height above ground and its altitude above mean sea level. An edge is dropped if its segment crosses a restricted polygon, or if the optional edge filter hook rejects it (the simulation module plugs its roof and terrain clearance check in here). After that we keep only the largest strongly connected component; otherwise a depot could end up on an island of nodes it cannot leave. Each remaining edge keeps its horizontal haversine length dh, its forward-azimuth bearing θ, its grade g = ∆alt/dh and its three-dimensional length d = q d2 h +∆alt2.

**p24** (96 words)

> 3.2.4 Risk terms that ignore the weather None of the static terms depend on the weather, so we compute them once per build. Each edge is walked in 25 m steps along the straight line between its endpoint altitudes, and at every step we read the flight altitude, the ground height, the tallest roof directly below and the tallest roof within 30 m. Table 3.1 lists all eleven risk components, static and weather-dependent, and the weight each gets in the score ρedge = clip ∑ wk ρk, 0, 1 , ρ(e;ω) = ρedge · dh s

**p25** (50 words)

> We write the edge’s energy proxy as ε = d 1+|g|+βh max(0,HW) Tailwind gets no credit, on purpose: that keeps ε ≥d for every edge, a bound the Tier 1 heuristic needs. For wind speed we use the 80 m forecast if there is one, since 80 m is near

**p26** (194 words)

> If not, the 10 m value is scaled up to cruise height by applied to this flight-height wind. Equation (3.4) makes edge costs depend on direction. In wind, u →v and v →u do not cost the same, which is why a symmetric lattice still needs a directed graph. 3.2.6 Weather risk and the go/no-go gate Only four of the eleven risk components react to weather: crosswind, gust excess, turbulence and the uniform rain-and-visibility term. Crosswind depends on edge heading, so one wind makes some corridors riskier than others, and turning the wind without changing its speed can shift the low-risk end of the front elsewhere on the map. apply weather(G, ω) re-costs each edge from its stored static features, so a fresh snapshot never forces the spatial layers to be rebuilt. On top of this sits the weather gate. We call a mission no-go if the WMO code reports a thunderstorm (95–99), rain reaches 7.5 mm/h, visibility drops under 1 km, or gusts pass 45 km/h, a provisional limit for an F450-class airframe. The reasons go onto the graph, and while any of them is present Tier 2 will not pick a route.

**p27** (122 words)

> Algorithm 1 Building and costing the lattice airspace graph (Module 1) Require: Region, box size, spacing s; DEM; buildings B; restricted zones Z; weather ω; weights wk; optional edge filter Ensure: Directed graph G = (V,E) with costs c(e) = [d,ε,ρ] and no-go reasons R 1: n ←round(box/s); place an n×n lattice in a local equirectangular frame 2: for each lattice point k do ▷node selection 14: dh ←haversine length; θ ←forward azimuth; g ←∆alt/dh; d ← q d2 h +∆alt2 15: sample e every 25 m; read flight altitude, ground height, roof below and roof within 30 m 16: store ρk for building, terrain, population, no-fly buffer, water and comm/GPS 17: end for compute ρk for wind, gust, weather and turbulence

**p28** (213 words)

> Module 2 computes the exact set of non-dominated routes between a source and a target. Its inputs are the costed graph from Module 1, the two end nodes and an expansion cap. It returns the Pareto front P as cost vectors with their node paths, plus some diagnostics: whether the cap cut the search short, the number of labels expanded and generated, and the runtime. 3.3.1 Labels and when one beats another Our engine implements NAMOA* [11]. A partial path is stored as a label (node,g,parent) in one flat table, and full paths are recovered at the end by following parent ids. Copying the path on every push would be the slowest step once labels run into the thousands, so the table keeps only the id. A new label g′ at node v is thrown away when some label already at v weakly dominates it, that is, is ≤on every objective up to a small tolerance. Identical vectors count as dominated, which keeps duplicates off the front. Labels the newcomer dominates are marked dead and skipped once popped. We need no visited set. Every edge has positive distance, so a path that comes back to a node is dominated by its own earlier label there, and the dominance check removes cycles by itself.

**p29** (223 words)

> 3.3.2 A heuristic that never overestimates For node v the heuristic is h(v) = [h, h, 0] with h the great-circle distance from v to the target. It is admissible on all three objectives. On distance, d ≥dh for every edge, and the horizontal lengths along any path to the target sum to at least h. On energy, Equation (3.5) gives ε ≥d, so the bound carries over. We prune a label once the current front weakly dominates g+h, and check this both when the label is generated and when it is popped, as the front can improve in between. Why order the open list by ∥g+h∥? Good routes then finish early, and each one tightens the pruning. 3.3.3 Capping the search and the front A cap on expansions stops a runaway search. The result is flagged as truncated only if some live label could still improve the front. An optional ε-dominance step can cap very large fronts at N routes, keeping the three single-objective extremes. We leave it off by default, so the benchmarks use the exact front. Algorithm 2 lists the whole procedure, with ⪯for weak dominance. 3.3.4 Why we chose exact search On a large graph a population method is usually quicker. We still went with an exact search. The same graph and weather always give the same front. Interior

**p31** (30 words)

> On each request the store lists the reasons the record cannot be reused : no record, a forced refresh, a different static graph, a different identity, expiry, or weather drift.

**p33** (117 words)

> Three weights come out of the tags, with no expert in the loop: rraw = 2F, draw = 3U, eraw = 2W, S = rraw +draw +eraw (3.8) wrisk = rraw Urgency gets a multiplier of 3 and the other two get 2, because we assume that in medical delivery urgency is the most sensitive of the three. Payload weight is tied to energy since a heavier load directly raises the power to hover and climb; there is no equally direct physical link from weight to risk or to distance. All three raw scores enter every weight. Raising F lifts wrisk and pulls the other two down together, so one tag change shifts the whole weight vector.

**p35** (12 words)

> where the exponent follows the way induced rotor power scales with weight.

**p36** (118 words)

> Routes that fail this are dropped before the argmax. The constants are flagged as placeholders in the code, and calibrated values from simulation will replace them. 3.5.6 HOLD advisories and the decision log When the weather gate gives any reason, or the battery rules out every route on the front, Tier 2 answers with a HOLD advisory and the reason instead of a route. In all other cases it returns the best-scoring feasible route. Algorithm 3 gives the full procedure. No search runs at this stage. The cost is one pass over the stored front, so the latency is a few milliseconds. Module 5 judges two things: the quality of a front and whether Tier 2 picks sensibly.

**p39** (142 words)

> 3.6.3 Sweeping all 225 tag sets The sweep runs Tier 2 on each of the 225 tag combinations. It counts how many distinct routes get picked and how many picks are non-extreme, and checks monotonicity. With the other tags held fixed, a higher F must never give a riskier route, a higher U never a longer one, and a higher W never one that uses more energy. We aim for zero violations. 3.6.4 Benchmarking as a loop We run benchmarking as a loop that repeats. Scenarios are fixed, every method runs, fronts are scored and Tier 2 picks checked, routes are flown in simulation, weak spots are studied, the model is changed, and the loop starts again. A full pass is four graphs times nine wind cases, with MOPSO at 100 particles, 200 iterations and 30 repetitions, roughly 21.6 million particle updates.

**p40** (83 words)

> 3.7.1 Writing the mission file Module 6 saves the selected route both as a QGC WPL 110 file and as a QGroundControl .plan file, merging consecutive waypoints that share bearing and slope. Route waypoints are in frame 3 (GLOBAL RELATIVE ALT), relative to home, so the exporter writes altrel = altAMSL −Hhome where Hhome is the ground elevation below the first waypoint. The height above the local ground is never written into frame 3, because that would be wrong wherever the terrain varies.

**p42** (22 words)

> Afterwards an evaluator works out the cross-track error, minimum clearance, any rule breach and how far from the pad the drone landed.


## Chapter 4 Results

**p43** (49 words)

> Here we cover the data behind the system, the hardware and software, the experiments done so far and what came out of them. The numbers have four sources: the first pass of the evaluation cycle (Section 3.6), a controlled wind experiment, the ablation runner and the first SITL flights.

**p44** (101 words)

> The obstacle density labels (5%, 20% and 40%) never reach the graph builder; the benchmark runner scales the final risk instead. So the scenarios really differ in size and seed, not in obstacle coverage, whatever the labels say. The next pass fixes this, and all benchmark numbers will be recomputed afterwards. 4.1.2 A lattice over a real region Four public data sources feed the real-region lattice (Table 4.2). Nothing in the method ties it to one place; our example is a coastal urban neighbourhood. The default 6 km box at 200 m spacing has 900 candidate nodes, of which 893 remain

**p45** (65 words)

> TABLE 4.2: Where each layer of the real-region lattice comes from Public Overpass servers now and then reset or time out. For offline runs that can be repeated, the project ships fixture layers: hand-made test data shaped like the real region. The ablation in Section 4.4.4 ran on these, so it shows how each layer affects the planner, not what the real world would measure.

**p48** (43 words)

> NAMOA* scores an IGD of zero in all three scenarios: every reference point is on its front. That is the least an exact method should manage, and it is also our best sign that the pruning rules never throw away a non-dominated route.

**p50** (42 words)

> A planner that used wind speed without direction would assign both cases the same cost. The two objectives also react to different things. Turn the wind round by 180◦ (0◦to 180◦, or 45◦to 225◦) and the minimum risk stays put, because crosswind

**p51** (109 words)

> Minimum energy does not: the headwind penalty cares which way along that axis the drone flies. Both effects fall straight out of Equation (3.4), and they are why headwind and crosswind are kept apart. The front itself changes shape. Calm air gives 36 routes; the eight winds give 6 to 14, and the four presets share just two distinct routes. Our reading is that in a steady wind, strong-crosswind edges are both dearer and riskier, so energy and risk start to move together and fewer genuine trade-offs remain. The fragile medical pick is the lowest-risk route on every windy front, but its energy depends on direction (26.87 to 33.63).

**p52** (159 words)

> North- and south-bound edges get the full crosswind and six times the risk score of east-west ones (0.225 against 0.038); the 0.038 on east-west edges is the gust term alone. The front on this flat lattice is still one route, though, because the straight diagonal wins on every objective. Wind by itself does not create trade-offs. It needs terrain, buildings or restricted zones to work with, and that is what the ablation looks at. 4.4.4 Taking out one layer at a time E4’s results are in Table 4.7. The runs used fixture layers, so read the numbers as the effect of each layer on the planner, not as real-world values. TABLE 4.7: One layer removed per run (6 km lattice, fixtures, 30 km/h SW wind) Switch it off and mean edge risk drops from 0.199 to 0.061, the safest route’s risk from 8.85 to 0.10, and minimum energy by 11%. Buildings raise the upper end of the risk range.

**p53** (206 words)

> TABLE 4.8: Routes Tier 2 picked on the S2 (16-route) and S3 (115-route) fronts On the 115-route S3 front, the four presets choose four different routes, and three of them are non-extreme points. Disaster relief takes the minimum-energy route, which matches its dominant weight. The emergency medicine pick deserves a closer look. It is only the 22nd-shortest route, yet it is just 1.9% longer than the shortest route on the front. Distance dominates its weights without owning them outright, so the utility gives up a sliver of distance to buy lower risk. A single-objective planner cannot make that trade, and a fixed-weight planner would make the same trade for every payload. How far apart Tier 2’s options are shows in the fragile medical and emergency picks. The fragile route runs 10.5% longer than the emergency one (41.02 against 37.11) and carries 71% less cumulative risk (1.92 against 6.56). It ranks 25th of 115 on risk, not 1st, because its distance and energy weights are above zero and the even safer routes are a lot longer. Mixed cargo sits between the two on every objective, which is what we hoped balanced weights would do. Each selection on S3 took about 0.14 ms; the target was one second.

**p54** (40 words)

> On the smaller S2 front, emergency medicine and disaster relief end up on the same minimum-energy route. We saw this in the wind experiment too. When the front is small, Tier 2 simply has fewer distinct routes to choose from.


## Chapter 5 Social Impact

**p57** (224 words)

> Think of a rural clinic waiting on blood or antivenom, a town surrounded by floodwater, or a hospital that sends samples to a central lab every day. Services like this already run, such as Zipline in Rwanda [9] and the hospital sample flights in Switzerland [2]. We add a way for each flight to fit its cargo. Urgent antivenom can go by the shortest corridor and a fragile sample by a calmer one, and the operator never has to understand the planner. That counts most where trained people are few. Whoever launches the drone at a district hospital or relief camp will probably be a nurse or a volunteer, not someone who plans routes for a living. The system asks them four plain questions about the payload (how fragile, how heavy, how urgent, how important) and works out the rest. Then there are the people underneath, who never agreed to a drone overhead. Every edge cost includes population exposure, restricted zones, water and nearness to buildings. Nodes above the legal ceiling are taken out, and the weather gate grounds flights in storms, heavy rain, low visibility or strong gusts. The planner will not cut across a busy market to save a few seconds. Each Tier 2 decision is logged with its tags, weights, route and any HOLD reason, so it can be replayed later.

**p58** (132 words)

> Until the risk weights are calibrated, risk numbers rank routes against each other; they are not probabilities of harm. Phase II outdoor flights will follow the Drone Rules, 2021 [13]. On short trips with light loads, a small electric drone takes the place of a van or motorcycle run and gives off no exhaust along the way. The planner helps in two ways. Its energy objective accounts for wind direction, which by itself shifted the cheapest route’s energy by 27.5% in our tests, so strong headwinds can be avoided. The payload weight tag also tilts the weights toward energy for heavy loads, exactly when battery use counts most. Routes that would dip into the 20% battery reserve are refused, which protects the mission and the battery’s lifespan. Computing needs are small, too.


## Chapter 6 Conclusion

**p59** (97 words)

> The first results are encouraging, with clear caveats. The Tier 1 front was exact in every benchmark scenario, while MOPSO found 38 of 115 routes on the largest. At a fixed speed, changing only the wind direction moved the cheapest route’s energy by 27.5%, and in SITL the drone stayed within 0.13 m RMS of the planned track. There are weak points too. In a steady wind the front shrank and the presets fell onto two routes; MOPSO ran at reduced settings; the ablation used fixture data; and the risk weights and battery constants are still placeholders.

**p60** (218 words)

> The next one runs the full benchmark (four graphs, nine wind cases, MOPSO at 100 particles, 200 iterations and 30 repetitions) with per-repetition statistics and a significance test. The density labels and the monotonicity grouping need fixing, and we want to find out why the front collapses in steady wind by trying other normalisations and weight multipliers. The ablation and Tier 2 studies will be redone on live map data, and Tier 2 will be set against crowding distance, knee point, shortest route and fixed weights over all 225 tag combinations. Two dashboard defects remain open, and Tier 1 has to get faster on large graphs. In simulation, the Gazebo world gets wind, the battery model gets calibrated for each payload weight class, and every preset gets flown in several wind cases with full flight logs. In Phase II the system moves to a Pixhawk-equipped F450 quadcopter carrying a Raspberry Pi 5 companion computer and a servo payload release. Tier 2 will run on the drone itself, and outdoor flights will measure how closely it holds the route and how long it takes from entering the tags to starting the mission. Later, the same two-tier idea might cover several drones sharing a front, multi-drop routes, or replanning in flight when the weather moves faster than the refresh window.


## References / Appendices

**p63** (6 words)

> All tunable constants live in config.py.

