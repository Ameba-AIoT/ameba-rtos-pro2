//------------------------------------------------------------------------------
// class name
//------------------------------------------------------------------------------
#include "class_name.h"

static const char *coco_names[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light",
									 "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
									 "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
									 "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
									 "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
									 "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
									 "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
									 "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
									};

static const char *voc_names_21[21] = {"__background__",
									   "aeroplane", "bicycle", "bird", "boat",
									   "bottle", "bus", "car", "cat", "chair",
									   "cow", "diningtable", "dog", "horse",
									   "motorbike", "person", "pottedplant",
									   "sheep", "sofa", "train", "tvmonitor"
									  };


static const char *voc_names_20[20] = {"aeroplane", "bicycle", "bird", "boat",
									   "bottle", "bus", "car", "cat", "chair",
									   "cow", "diningtable", "dog", "horse",
									   "motorbike", "person", "pottedplant",
									   "sheep", "sofa", "train", "tvmonitor"
									  };

static const char *audioset_names[521] = {"Speech", "Child speech, kid speaking", "Conversation", "Narration, monologue", "Babbling",
										  "Speech synthesizer", "Shout", "Bellow", "Whoop", "Yell", "Children shouting", "Screaming",
										  "Whispering", "Laughter", "Baby laughter", "Giggle", "Snicker", "Belly laugh", "Chuckle, chortle",
										  "Crying, sobbing", "Baby cry, infant cry", "Whimper", "Wail, moan", "Sigh", "Singing", "Choir",
										  "Yodeling", "Chant", "Mantra", "Child singing", "Synthetic singing", "Rapping", "Humming",
										  "Groan", "Grunt", "Whistling", "Breathing", "Wheeze", "Snoring", "Gasp", "Pant", "Snort",
										  "Cough", "Throat clearing", "Sneeze", "Sniff", "Run", "Shuffle", "Walk, footsteps", "Chewing, mastication",
										  "Biting", "Gargling", "Stomach rumble", "Burping, eructation", "Hiccup", "Fart", "Hands", "Finger snapping",
										  "Clapping", "Heart sounds, heartbeat", "Heart murmur", "Cheering", "Applause", "Chatter", "Crowd", "Hubbub, speech noise, speech babble",
										  "Children playing", "Animal", "Domestic animals, pets", "Dog", "Bark", "Yip", "Howl", "Bow-wow",
										  "Growling", "Whimper (dog)", "Cat", "Purr", "Meow", "Hiss", "Caterwaul", "Livestock, farm animals, working animals",
										  "Horse", "Clip-clop", "Neigh, whinny", "Cattle, bovinae", "Moo", "Cowbell", "Pig", "Oink", "Goat",
										  "Bleat", "Sheep", "Fowl", "Chicken, rooster", "Cluck", "Crowing, cock-a-doodle-doo", "Turkey", "Gobble",
										  "Duck", "Quack", "Goose", "Honk", "Wild animals", "Roaring cats (lions, tigers)", "Roar", "Bird",
										  "Bird vocalization, bird call, bird song", "Chirp, tweet", "Squawk", "Pigeon, dove", "Coo", "Crow",
										  "Caw", "Owl", "Hoot", "Bird flight, flapping wings", "Canidae, dogs, wolves", "Rodents, rats, mice", "Mouse",
										  "Patter", "Insect", "Cricket", "Mosquito", "Fly, housefly", "Buzz", "Bee, wasp, etc.", "Frog", "Croak", "Snake", "Rattle",
										  "Whale vocalization", "Music", "Musical instrument", "Plucked string instrument", "Guitar", "Electric guitar", "Bass guitar",
										  "Acoustic guitar", "Steel guitar, slide guitar", "Tapping (guitar technique)", "Strum", "Banjo", "Sitar", "Mandolin", "Zither",
										  "Ukulele", "Keyboard (musical)", "Piano", "Electric piano", "Organ", "Electronic organ", "Hammond organ", "Synthesizer", "Sampler",
										  "Harpsichord", "Percussion", "Drum kit", "Drum machine", "Drum", "Snare drum", "Rimshot", "Drum roll", "Bass drum", "Timpani", "Tabla",
										  "Cymbal", "Hi-hat", "Wood block", "Tambourine", "Rattle (instrument)", "Maraca", "Gong", "Tubular bells", "Mallet percussion", "Marimba, xylophone",
										  "Glockenspiel", "Vibraphone", "Steelpan", "Orchestra", "Brass instrument", "French horn", "Trumpet", "Trombone", "Bowed string instrument",
										  "String section", "Violin, fiddle", "Pizzicato", "Cello", "Double bass", "Wind instrument, woodwind instrument", "Flute", "Saxophone",
										  "Clarinet", "Harp", "Bell", "Church bell", "Jingle bell", "Bicycle bell", "Tuning fork", "Chime", "Wind chime", "Change ringing (campanology)",
										  "Harmonica", "Accordion", "Bagpipes", "Didgeridoo", "Shofar", "Theremin", "Singing bowl", "Scratching (performance technique)",
										  "Pop music", "Hip hop music", "Beatboxing", "Rock music", "Heavy metal", "Punk rock", "Grunge", "Progressive rock", "Rock and roll",
										  "Psychedelic rock", "Rhythm and blues", "Soul music", "Reggae", "Country", "Swing music", "Bluegrass", "Funk", "Folk music", "Middle Eastern music",
										  "Jazz", "Disco", "Classical music", "Opera", "Electronic music", "House music", "Techno", "Dubstep", "Drum and bass", "Electronica", "Electronic dance music", "Ambient music",
										  "Trance music", "Music of Latin America", "Salsa music", "Flamenco", "Blues", "Music for children", "New-age music", "Vocal music",
										  "A capella", "Music of Africa", "Afrobeat", "Christian music", "Gospel music", "Music of Asia", "Carnatic music", "Music of Bollywood", "Ska", "Traditional music",
										  "Independent music", "Song", "Background music", "Theme music", "Jingle (music)", "Soundtrack music", "Lullaby", "Video game music", "Christmas music",
										  "Dance music", "Wedding music", "Happy music", "Sad music", "Tender music", "Exciting music", "Angry music", "Scary music", "Wind", "Rustling leaves",
										  "Wind noise (microphone)", "Thunderstorm", "Thunder", "Water", "Rain", "Raindrop", "Rain on surface", "Stream", "Waterfall", "Ocean", "Waves, surf",
										  "Steam", "Gurgling", "Fire", "Crackle", "Vehicle", "Boat, Water vehicle", "Sailboat, sailing ship", "Rowboat, canoe, kayak", "Motorboat, speedboat",
										  "Ship", "Motor vehicle (road)", "Car", "Vehicle horn, car horn, honking", "Toot", "Car alarm", "Power windows, electric windows", "Skidding", "Tire squeal",
										  "Car passing by", "Race car, auto racing", "Truck", "Air brake", "Air horn, truck horn", "Reversing beeps", "Ice cream truck, ice cream van", "Bus",
										  "Emergency vehicle", "Police car (siren)", "Ambulance (siren)", "Fire engine, fire truck (siren)", "Motorcycle", "Traffic noise, roadway noise", "Rail transport",
										  "Train", "Train whistle", "Train horn", "Railroad car, train wagon", "Train wheels squealing", "Subway, metro, underground", "Aircraft", "Aircraft engine", "Jet engine",
										  "Propeller, airscrew", "Helicopter", "Fixed-wing aircraft, airplane", "Bicycle", "Skateboard", "Engine", "Light engine (high frequency)", "Dental drill, dentist's drill",
										  "Lawn mower", "Chainsaw", "Medium engine (mid frequency)", "Heavy engine (low frequency)", "Engine knocking", "Engine starting", "Idling", "Accelerating, revving, vroom",
										  "Door", "Doorbell", "Ding-dong", "Sliding door", "Slam", "Knock", "Tap", "Squeak", "Cupboard open or close", "Drawer open or close",
										  "Dishes, pots, and pans", "Cutlery, silverware", "Chopping (food)", "Frying (food)", "Microwave oven", "Blender", "Water tap, faucet",
										  "Sink (filling or washing)", "Bathtub (filling or washing)", "Hair dryer", "Toilet flush", "Toothbrush", "Electric toothbrush", "Vacuum cleaner", "Zipper (clothing)",
										  "Keys jangling", "Coin (dropping)", "Scissors", "Electric shaver, electric razor", "Shuffling cards", "Typing", "Typewriter", "Computer keyboard", "Writing",
										  "Alarm", "Telephone", "Telephone bell ringing", "Ringtone", "Telephone dialing, DTMF", "Dial tone", "Busy signal", "Alarm clock", "Siren", "Civil defense siren",
										  "Buzzer", "Smoke detector, smoke alarm", "Fire alarm", "Foghorn", "Whistle", "Steam whistle", "Mechanisms", "Ratchet, pawl", "Clock", "Tick", "Tick-tock", "Gears",
										  "Pulleys", "Sewing machine", "Mechanical fan", "Air conditioning", "Cash register", "Printer", "Camera", "Single-lens reflex camera", "Tools", "Hammer", "Jackhammer",
										  "Sawing", "Filing (rasp)", "Sanding", "Power tool", "Drill", "Explosion", "Gunshot, gunfire", "Machine gun", "Fusillade", "Artillery fire", "Cap gun", "Fireworks",
										  "Firecracker", "Burst, pop", "Eruption", "Boom", "Wood", "Chop", "Splinter", "Crack", "Glass", "Chink, clink", "Shatter", "Liquid", "Splash, splatter", "Slosh",
										  "Squish", "Drip", "Pour", "Trickle, dribble", "Gush", "Fill (with liquid)", "Spray", "Pump (liquid)", "Stir", "Boiling", "Sonar", "Arrow", "Whoosh, swoosh, swish",
										  "Thump, thud", "Thunk", "Electronic tuner", "Effects unit", "Chorus effect", "Basketball bounce", "Bang", "Slap, smack", "Whack, thwack", "Smash, crash", "Breaking",
										  "Bouncing", "Whip", "Flap", "Scratch", "Scrape", "Rub", "Roll", "Crushing", "Crumpling, crinkling", "Tearing", "Beep, bleep", "Ping", "Ding", "Clang", "Squeal", "Creak",
										  "Rustle", "Whir", "Clatter", "Sizzle", "Clicking", "Clickety-clack", "Rumble", "Plop", "Jingle, tinkle", "Hum", "Zing", "Boing", "Crunch", "Silence", "Sine wave", "Harmonic",
										  "Chirp tone", "Sound effect", "Pulse", "Inside, small room", "Inside, large room or hall", "Inside, public space", "Outside, urban or manmade", "Outside, rural or natural",
										  "Reverberation", "Echo", "Noise", "Environmental noise", "Static", "Mains hum", "Distortion", "Sidetone", "Cacophony", "White noise", "Pink noise", "Throbbing", "Vibration",
										  "Television", "Radio", "Field recording"
										 };

#define voc_names   voc_names_20

const char *coco_name_get_by_id(int id)
{
	return coco_names[id];
}

const char *voc_name_get_by_id(int id)
{
	return voc_names[id];
}

const char *audioset_name_get_by_id(int id)
{
	return audioset_names[id];
}
