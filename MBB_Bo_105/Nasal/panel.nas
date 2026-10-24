# Instrument panel systems (Models/Panel, fdm/cockpit): what the instruments show that the FDM does not compute,
# and the warning light segments. Flight manual BO 105 CB-5/CBS-5: markings section 2.18, warning lights section 3.2.1
# and fig. 7-19, mast moment indicator 2.17.3 and 7.7.5. The aircraft has no electrical system: the panel is powered.

var P = "/instrumentation/bo105/";
var W = P ~ "warning/";
var eng = [props.globals.getNode("/engines/engine[0]", 1), props.globals.getNode("/engines/engine[1]", 1)];
var gear = props.globals.getNode("/rotors/gear", 1);
var dt = 0.1;

var lag = func(node, name, target, tau) {
	var v = node.getValue(name);
	if (v == nil) v = target;
	node.setDoubleValue(name, v + (target - v) * math.min(1, dt / tau));
}

var clamp = func(x, lo, hi) { math.max(lo, math.min(hi, x)) }

# temperatures start at the ambient's
var oat = getprop("/environment/temperature-degc") or 15;
foreach (var e; eng) e.setDoubleValue("oil-temp-degc", oat);
gear.setDoubleValue("oil-temp-degc", oat);

# mast moment indicator: the hub moment (FDM), damped; TEST drives the needle to the scale's end, then back to the
# white calibration line; the LIMIT light comes on with power, latches at the yellow arc's end, and goes off when TEST
# is released below it
var mmi = props.globals.getNode("/instrumentation/mmi", 1);
mmi.setBoolValue("limit-light", 1);
mmi.setBoolValue("test", 0);
mmi.setDoubleValue("indicated-pct", 0);
var test_t = 0;
setlistener("/instrumentation/mmi/test", func(n) {
	if (n.getBoolValue()) test_t = 0;
	elsif (mmi.getValue("indicated-pct") < 100) mmi.setBoolValue("limit-light", 0);
}, 0, 0);

var update = func {
	oat = getprop("/environment/temperature-degc") or 15;
	var nr = getprop("/rotors/main/rpm-pct") or 0;
	var n1 = [];
	var tq = 0;
	forindex (var k; eng) {
		var e = eng[k];
		var n = e.getValue("n1-pct") or 0;
		append(n1, n);
		tq += (e.getValue("torque-pct") or 0) / 2;
		# engine oil: pressure with N1 (3.7 kp/cm2 at ground idle, 8.4 at 100 %), temperature towards ~90 degC
		lag(e, "oil-pressure-kpcm2", 8.4 * math.pow(clamp(n / 100, 0, 1.1), 1.6), 1.0);
		var hot = n > 10 ? oat + 45 + 30 * n / 100 : oat;
		lag(e, "oil-temp-degc", hot, hot > e.getValue("oil-temp-degc") ? 240 : 600);
		# fuel pressure ahead of the engine's filter: the main tank pumps', less a little with the flow
		var fuel = (getprop("/consumables/fuel/tank[1]/level-lbs") or 0) > 1;
		lag(e, "fuel-press-kpcm2", fuel ? 1.15 - 0.0008 * (e.getValue("fuel-flow-pph") or 0) : 0, 0.5);
	}
	# main transmission oil: pressure with rotor speed, temperature with rotor speed and torque
	lag(gear, "oil-pressure-kpcm2", 1.6 * math.pow(clamp(nr / 100, 0, 1.1), 2), 1.0);
	var xt = oat + 55 * clamp(nr / 100, 0, 1.1) * (0.5 + 0.5 * tq / 100);
	lag(gear, "oil-temp-degc", xt, xt > gear.getValue("oil-temp-degc") ? 300 : 900);

	# mast moment
	if (mmi.getBoolValue("test")) {
		test_t += dt;
		lag(mmi, "indicated-pct", test_t < 0.6 ? 130 : 69.2, 0.15);
	} else {
		lag(mmi, "indicated-pct", getprop("/rotors/main/mast-moment-pct") or 0, 0.3);
		if (mmi.getValue("indicated-pct") >= 100) mmi.setBoolValue("limit-light", 1);
	}

	lights();

	# warning light segments (fig. 7-19); the failures the simulation does not have stay dark
	var flash = math.mod(int((getprop("/sim/time/elapsed-sec") or 0) * 3), 2);	# the RPM light flashes at 1.5 Hz
	var test = getprop(P ~ "lamp-test") or 0;
	var on = func(name, c) setprop(W ~ name, test or c);
	var split = math.abs(n1[0] - n1[1]) >= 12;
	on("rpm", (nr <= 95 or nr >= 103 or split) and flash);
	on("t-oil", gear.getValue("oil-pressure-kpcm2") < 0.5 or gear.getValue("oil-temp-degc") >= 105);
	on("low-fuel", (getprop("/consumables/fuel/tank[1]/level-lbs") or 0) < 60 / 0.45359);
	on("gen-1", n1[0] < 55);
	on("gen-2", n1[1] < 55);
	on("hyd-1", nr < 60);
	on("hyd-2", nr < 60);
	foreach (var n; ["hy-block", "filt-1", "filt-2", "mag-plug-1", "mag-plug-2", "bat-60", "bat-70", "oil-cool"])
		on(n, 0);
	# engine fire warning lights: their TEST FIRE pushbuttons
	setprop(W ~ "fire1", getprop(P ~ "test-fire[0]") or 0);
	setprop(W ~ "fire2", getprop(P ~ "test-fire[1]") or 0);
}

# ---------------------------------------------------------------- cockpit lights
# FM 7.12.3-7.12.6, with the NVG compatible instrument lighting of FM supplement 10-52 (NVIS Green A, as the UH-60
# model's; the supplement's own text is not at hand): the instruments' lighted bezels on the INSTR LIGHT switch (centre
# console) and the INSTR LIGHT rheostat (instrument panel), /controls/lighting/instr-light and instruments-norm; the
# whip light on its own switch (whip-light; not dimmed); the utility light's OFF-BRT rheostat (utility-norm); the
# cabin's ceiling light, INTERNAL LIGHT (internal-light). The warning and caution lights are bright, or dim
# (warn-dim, at warn-dim-norm: DIM_LO to DIM_HI over its travel) for the night. The radio's display follows the light
# outside, as a photocell would. Aircraft menu > Cockpit lights (Dialogs/lights.xml). Levels, 0-1, in P ~ "lights/" for
# the lamps (Models/Panel/Instruments, Models/Lights/interior.xml), and in cd/m2 in "lights/hdr/" for what emits through
# Models/Effects/Lit (fdm/cockpit/gauges.py: the lamps' LAMP_BRT, 1030 cd/m2, sunlight readable). The aircraft has no
# electrical system: everything is powered. A start with the sun down has the instrument lights on at half and the
# warning lights dim; by day they are off and bright.
var LT = P ~ "lights/";
var LAMP_BRT = 1030;
var DIM_LO = 0.05; var DIM_HI = 2;
var DISPLAY_NIGHT = 3; var DISPLAY_DAY = 309;
var ctl = func(name, default = 0) { var v = getprop("/controls/lighting/" ~ name); v == nil ? default : v; };

var lights = func {
	var instr = ctl("instr-light") ? clamp(ctl("instruments-norm"), 0, 1) : 0;
	setprop(LT ~ "instruments", instr);
	var warn = ctl("warn-dim") ? DIM_LO * math.pow(DIM_HI / DIM_LO, clamp(ctl("warn-dim-norm", 0.5), 0, 1)) : LAMP_BRT;
	setprop(LT ~ "hdr/warn", warn);
	setprop(LT ~ "hdr/mmi-limit", mmi.getBoolValue("limit-light") ? warn : 0);
	var day = clamp((90 - (getprop("/sim/time/sun-angle-rad") or 0) * R2D + 6) / 12, 0, 1);	# sun 6 deg below to above
	setprop(LT ~ "hdr/displays", DISPLAY_NIGHT * math.pow(DISPLAY_DAY / DISPLAY_NIGHT, day));
	setprop(LT ~ "whip", ctl("whip-light") ? 1 : 0);
	setprop(LT ~ "utility", clamp(ctl("utility-norm"), 0, 1));
	setprop(LT ~ "cabin", ctl("internal-light") ? 1 : 0);
}

var lights_dialog = gui.Dialog.new("/sim/gui/dialogs/bo105-lights/dialog", "Aircraft/MBB_Bo_105/Dialogs/lights.xml");

# the start's lights, once its time of day has taken (it hasn't when this runs)
settimer(func {
	var night = (getprop("/sim/time/sun-angle-rad") or 0) * R2D > 90;
	var init = func(name, v) if (getprop("/controls/lighting/" ~ name) == nil) setprop("/controls/lighting/" ~ name, v);
	init("instr-light", night);
	# (FlightGear itself declares the rheostat, at 0: at night it starts at half unless set)
	if (night and !(getprop("/controls/lighting/instruments-norm") or 0)) setprop("/controls/lighting/instruments-norm", 0.5);
	init("warn-dim", night);
	init("warn-dim-norm", 0.5);
	init("whip-light", 0);
	init("utility-norm", 0);
	init("internal-light", 0);
}, 5);

var timer = maketimer(dt, update);
timer.start();
