//Initializes the HTML canvas API drawing context
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

//Initializes canvas constants
const CANVAS_WIDTH = canvas.width = 1700;
const CANVAS_HEIGHT = canvas.height = 925;
const MARGIN = 40;
const LEFT_MARGIN = MARGIN;
const RIGHT_MARGIN = CANVAS_WIDTH - MARGIN;
const TOP_MARGIN = MARGIN;
const BOTTOM_MARGIN = CANVAS_HEIGHT - MARGIN;

//Initializes boid behavior constants
const TURN_FACTOR = 3;
const MAX_SPEED = 250;
const MIN_SPEED = 125;
const VISION_RADIUS = 35;
const PROTECTED_RADIUS = 9;
const CENTER = 0.0005;
const AVOID = 0.25;
const MATCH = 0.05;
const RANDOMNESS = 5;

//Initializes delta time
var dt = 1/60;

//Class defining 2D vectors and their methods
class Vector{
    constructor(x, y){
        this.x = x;
        this.y = y;
    }

    add(vec){
        return new Vector(this.x + vec.x, this.y + vec.y);
    }

    subtr(vec){
        return new Vector(this.x - vec.x, this.y - vec.y);
    }

    mult(n){
        return new Vector(this.x * n, this.y * n);
    }

    mag(){
        return Math.sqrt(this.x**2 + this.y**2);
    }

    dist(vec){
        return this.subtr(vec).mag();
    }

    unit(){
        if(this.mag() == 0){
            return new Vector(0,0);
        } else {
            return new Vector(this.x / this.mag(), this.y / this.mag());
        }
    }

    normal(){
        return new Vector(-this.y, this.x);
    }
}

//Matrix class
class Matrix{
    constructor(rows, cols){
        this.rows = rows;
        this.cols = cols;
        this.data = [];

        for (let i = 0; i < this.rows; i++){
            this.data[i] = [];
            for (let j = 0; j < this.cols; j++){
                this.data[i][j] = [];
            }
        }
    }

    clear(){
        for (let i = 0; i < this.rows; i++){
            this.data[i] = [];
            for (let j = 0; j < this.cols; j++){
                this.data[i][j] = [];
            }
        }
    }
}

//Creates boid class
class Boid{
    constructor(pos){
        this.pos = pos;
        this.vel = new Vector(0,0);

        this.isScout1 = false;
        this.isScout2 = false;
        this.biasVal = 0.4;

        //Adds this boid to the list of all boids upon creation
        BOIDS.push(this);
    }

    addToPartition(){
        GRID.data[Math.floor(this.pos.x/VISION_RADIUS)+1][Math.floor(this.pos.y/VISION_RADIUS)+1].push(this);
    }

    //Draws triangle oriented in this boid's direction
    draw(){
        ctx.beginPath();
        let vert1 = this.pos.add(this.vel.unit().mult(6));
        ctx.moveTo(vert1.x, vert1.y);
        let vert2 = this.pos.subtr(this.vel.unit().mult(3)).add(this.vel.unit().normal().mult(3));
        ctx.lineTo(vert2.x, vert2.y);
        let vert3 = this.pos.subtr(this.vel.unit().mult(3)).subtr(this.vel.unit().normal().mult(3));
        ctx.lineTo(vert3.x, vert3.y);
        ctx.closePath();
        ctx.fillStyle = 'rgb(19, 14, 10)';
        ctx.fill();
    }
}

//Rounds number to given precision
function round(number, precision){
    let factor = 10**precision;
    return Math.round(number * factor) / factor;
}

//Generates random number given a min and max value
function random(min, max){
    return Math.random() * ((max-1)-min+1) + min;
}

//Displays text
function testText(text, y){
    ctx.fillStyle = 'rgb(0, 0, 0)';
    ctx.fillText(text, 15, y);
}

//Describes boid behavior for any given state each frame
function physics(){
    GRID.clear();
    BOIDS.forEach(b => {
        b.addToPartition();
    });

    for(let i = 0; i < GRID.rows; i++){
        for(let j = 0; j < GRID.cols; j++){
            if(GRID.data[i][j][0] == undefined){
                continue;
            }
            let iForward = (i+1)%GRID.rows;
            let iBack = (i+GRID.rows-1)%GRID.rows;
            let jForward = (j+1)%GRID.cols;
            let jBack = (j+GRID.cols-1)%GRID.cols;
            let boidsToCheck = GRID.data[i][j].concat(
                GRID.data[iForward][jForward],
                GRID.data[iForward][j],
                GRID.data[iForward][jBack],
                GRID.data[i][jBack],
                GRID.data[iBack][jBack],
                GRID.data[iBack][j],
                GRID.data[iBack][jForward],
                GRID.data[i][jForward]
            );
            let l = boidsToCheck.length;
            GRID.data[i][j].forEach(b => {

                //Zeros all relevant variables
                let avoidForce = new Vector(0,0);
                let averageVel = new Vector(0,0);
                let averagePos = new Vector(0,0);
                let nearbyBoids = 0;

                for(let o = 0; o < l; o++){
                    let otherBoid = boidsToCheck[o];
                    if(otherBoid == b){
                        continue;
                    }

                    //Gets this boid's distance to other boids and checks if it is within this boid's vision
                    let dist = b.pos.dist(otherBoid.pos);
                    if(dist < PROTECTED_RADIUS){

                        //Calculates the avoidance force for this boid
                        avoidForce = avoidForce.add(b.pos.subtr(otherBoid.pos));
                    } else if(dist < VISION_RADIUS){

                        //Calculates the total position and velocity of nearby boids
                        averageVel = averageVel.add(otherBoid.vel);
                        averagePos = averagePos.add(otherBoid.pos);

                        //Counts nearby boids
                        nearbyBoids++;
                    }
                }

                //Calculates the average force and position of nearby boids
                if(nearbyBoids > 0){
                    averageVel = averageVel.mult(1/nearbyBoids);
                    averagePos = averagePos.mult(1/nearbyBoids);
                }

                //Apples matching, centering, and avoiding forces to each boid as well as a random force
                b.vel = b.vel.add(new Vector(random(-1,1),random(-1,1)).unit().mult(random(0,RANDOMNESS)));
                b.vel = b.vel.add(averageVel.subtr(b.vel).mult(MATCH));
                b.vel = b.vel.add(averagePos.subtr(b.pos).mult(CENTER));
                b.vel = b.vel.add(avoidForce.mult(AVOID));

                if(b.isScout1){
                    let biasForce = FOOD_POS1.subtr(b.pos).unit();
                    b.vel = b.vel.mult(1-b.biasVal).add(biasForce.mult(b.biasVal));
                }

                if(b.isScout2){
                    let biasForce = FOOD_POS2.subtr(b.pos).unit();
                    b.vel = b.vel.mult(1-b.biasVal).add(biasForce.mult(b.biasVal));
                }

                //Keeps boids speed between the min and max speed
                if(b.vel.mag() < MIN_SPEED){
                    b.vel = b.vel.unit().mult(MIN_SPEED);
                }
                if(b.vel.mag() > MAX_SPEED){
                    b.vel = b.vel.unit().mult(MAX_SPEED);
                }
            });
        }
    }

    ctx.clearRect(0,0,CANVAS_WIDTH,CANVAS_HEIGHT);
    BOIDS.forEach(b => {
        //Adds each boid's velocity to its position
        b.pos = b.pos.add(b.vel.mult(dt));

        //Applies a force if boids get to close to the edge
        if(b.pos.x < LEFT_MARGIN){
            b.vel.x += TURN_FACTOR;
            if(b.pos.x < 0){
                b.pos.x = 0;
                b.vel.x *= -1;
            }
        }
        if(b.pos.x > RIGHT_MARGIN){
            b.vel.x -= TURN_FACTOR;
            if(b.pos.x > CANVAS_WIDTH){
                b.pos.x = CANVAS_WIDTH;
                b.vel.x *= -1;
            }
        }
        if(b.pos.y > BOTTOM_MARGIN){
            b.vel.y -= TURN_FACTOR;
            if(b.pos.y > CANVAS_HEIGHT){
                b.pos.y = CANVAS_HEIGHT;
                b.vel.y *= -1;
            }
        }
        if(b.pos.y < TOP_MARGIN){
            b.vel.y += TURN_FACTOR;
            if(b.pos.y < 0){
                b.pos.y = 0;
                b.vel.y *= -1;
            }
        }

        b.draw();
    });
}

function updateDeltaT(){
    //Calculates fps and delta time
    end = Date.now();
    let fps = 1000/(end-start);
    if(fps === Infinity){
        fps = 250;
    }
    dt = (1/fps);
    start = Date.now();
}

function reset(){
    //Restarts the simulation over
    BOIDS = [];
    dt = 1/60;
    start = Date.now();
    end;
    BOIDS = [];
    for(let i = 0; i < NUM_BOIDS; i++){
        new Boid(new Vector(random(0,CANVAS_WIDTH),random(0,CANVAS_HEIGHT)));
    }
}

function userInput(){
    //Restarts the simulation over if the r key is pressed
    addEventListener('keydown', e => {
        if(e.key == 'r'){
            reset();
        }
        if(e.key == 'p'){
            BOIDS[0] = 1;
        }
    });
}

//Main loop
function main(){
    physics();
    updateDeltaT();
}

//Initialize boids and fps stuff
var start = Date.now();
var end;
const NUM_BOIDS = 500;
const NUM_SCOUT1S = 100;
const NUM_SCOUT2S = 100;
const FOOD_POS1 = new Vector(CANVAS_WIDTH/3,CANVAS_HEIGHT/2);
const FOOD_POS2 = new Vector(2*CANVAS_WIDTH/3,CANVAS_HEIGHT/2);
var BOIDS = [];
var GRID = new Matrix(Math.ceil(CANVAS_WIDTH/VISION_RADIUS)+2,Math.ceil(CANVAS_HEIGHT/VISION_RADIUS)+2);
for(let i = 0; i < NUM_BOIDS; i++){
    new Boid(new Vector(random(0,CANVAS_WIDTH),random(0,CANVAS_HEIGHT)));
}
for(let i = 0; i < NUM_SCOUT1S; i++){
    BOIDS[i].isScout1 = true;
}
for(let i = 0; i < NUM_SCOUT2S; i++){
    BOIDS[NUM_BOIDS-1-i].isScout2 = true;
}

//Run main loop
userInput();
setInterval(main);