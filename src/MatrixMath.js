import rootFinding from 'root-finding';
import algos from 'algos';
import _ from 'underscore';

function MatrixMath() {
}

function StraightLine(x, y, hdg, length){
    this.x = x;
    this.y = y;
    this.hdg = hdg;
    this.length = length;
    this.type = "StraightLine";
}

StraightLine.prototype = Object.assign(Object.create(StraightLine.prototype));

function ArcLine(x, y, hdg, cur, length){
    this.x = x;
    this.y = y;
    this.hdg = hdg;
    this.cur = cur;
    this.length = length;
    this.type = "ArcLine";
}

ArcLine.prototype = Object.assign(Object.create(ArcLine.prototype));

function StraightSideLine(x, y, hdg, length, poly) {
    this.x = x;
    this.y = y;
    this.hdg = hdg;
    this.length = length;
    this.poly = poly;
    this.type = "StraightSideLine";
}

StraightSideLine.prototype = Object.assign(Object.create(StraightSideLine.prototype));

function ArcSideLine(x, y, hdg, cur, length, poly) {
    this.x = x;
    this.y = y;
    this.hdg = hdg;
    this.curvature = cur;
    this.length = length;
    this.poly = poly;
    this.type = "ArcSideLine";
    this.t = 0; //t取值范围为0-length
    this.getPointX = function(){
        return this.x - (1 / this.curvature) * Math.sin(this.hdg) + (this.poly.value(this.t) - (1 / this.curvature)) * Math.sin(-this.hdg - this.t * this.curvature);
    }
    this.getPointY = function(){
        return this.y + (1 / this.curvature) * Math.cos(this.hdg) + (this.poly.value(this.t) - (1 / this.curvature)) * Math.cos(-this.hdg - this.t * this.curvature);
    }
}

ArcSideLine.prototype = Object.assign(Object.create(ArcSideLine.prototype));

function StraightSideLineCrossStraightSideLine(line1, line2) {
    let x0 = line1.x;
    let y0 = line1.y;
    let x1 = line2.x;
    let y1 = line2.y;
    let alpha = line1.hdg;
    let beta = line2.hdg;
    let poly0 = line1.poly;
    let poly1 = line2.poly;

    function f(s) {//方程组拆解消元后获得 t = f(s) 方程组
        return (x1 * Math.cos(alpha) + s * Math.cos(beta) * Math.cos(alpha) - poly1.value(s) * Math.sin(beta) * Math.cos(alpha)
            + y1 * Math.sin(alpha) + s * Math.sin(beta) * Math.sin(alpha) + poly1.value(s) * Math.cos(beta) * Math.sin(alpha)
            - x0 * Math.cos(alpha) - y0 * Math.sin(alpha))
            / (Math.pow(Math.cos(alpha), 2) + Math.pow(Math.sin(alpha), 2));
    }

    function g(t) {//方程组拆解消元后获得 s = f(t) 方程组
        return (x0 * Math.cos(beta) + t * Math.cos(alpha) * Math.cos(beta) - poly0.value(t) * Math.sin(alpha) * Math.cos(beta)
            + y0 * Math.sin(beta) + t * Math.sin(alpha) * Math.sin(beta) + poly0.value(t) * Math.cos(alpha) * Math.sin(beta)
            - x1 * Math.cos(beta) - y1 * Math.sin(beta))
            / (Math.pow(Math.cos(beta), 2) + Math.pow(Math.sin(beta), 2));
    }

    function co(t) {
        return f(g(t)) - t;
    }

    this.cross = function () {
        let t = algos.secantSolver(co, 0);
        if (t) {
            let gt = g(t);
            if (t >= 0 && t <= line1.length && gt >= 0 && gt <= line2.length) {
                return {
                    t: t,
                    s: gt
                };
            }
        }
    }
}

function ArcSideLineCrossArcSideLine(arc1, arc2) {
    let x0 = arc1.x;
    let y0 = arc1.y;
    let x1 = arc2.x;
    let y1 = arc2.y;
    let cur0 = arc1.curvature;
    let cur1 = arc2.curvature;
    let alpha = arc1.hdg;
    let beta = arc2.hdg;
    let poly0 = arc1.poly;
    let poly1 = arc2.poly;

    function f(m) {
        return function(s){
            return (-alpha - Math.atan((x1 - (1 / cur1) * Math.sin(beta) + (poly1.value(s) - 1 / cur1) * Math.sin(-1 * beta - s * cur1) - x0 + (1 / cur0) * Math.sin(alpha))
            / (y1 + (1 / cur1) * Math.cos(beta) + (poly1.value(s) - 1 / cur1) * Math.cos(-1 * beta - s * cur1) - y0 - (1 / cur0) * Math.cos(alpha))) + Math.PI * m)
            / cur0;
        }
    }

    function co(m) {
        return function(s){
            return x1 - (1 / cur1) * Math.sin(beta) + (poly1.value(s) - 1 / cur1) * Math.sin(-1 * beta - s * cur1)
            - x0 + (1 / cur0) * Math.sin(alpha) - (poly0.value(f(m)(s)) - 1 / cur0) * Math.sin(-1 * alpha - f(m)(s) * cur0);
        }
    }

    function mod(x, a) {
        let b = x % a;
        if (b >= 0) return b;
        return a + b;
    }
    function checkResult(t, s) {
        if(s && t >= 0 && t <= arc1.length && s >= 0 && s <= arc2.length){
            arc1.t = t;
            arc2.t = s;
            if( Math.abs(arc1.getPointX() - arc2.getPointX()) < 0.1 && Math.abs(arc1.getPointY() - arc2.getPointY()) < 0.1){
                return true;
            }
        }
        console.debug("[MatrixMath] t & s is not satisfied ");
        return false;
    }
    this.cross = function () {
        let result = [];
        let cos = [co(0), co(1), co(-1), co(2), co(-2), co(3), co(-3), co(4), co(-4), co(5), co(-5)];
        let fs = [f(0), f(1), f(-1), f(2), f(-2), f(3), f(-3), f(4), f(-4), f(5), f(-5)];
        let guess = [0, arc2.length/4, arc2.length/2, arc2.length * 3/4, arc2.length];
        for (let i = 0; i < cos.length; i++) {
            for(let j = 0; j< guess.length; j++){
                let s = algos.secantSolver(cos[i], guess[j]);
                // let s = rootFinding.newtonRaphson(guess[j], 0.01, 10, 0.01, cos[i]);
                if (!s) continue;
                s = mod(s, Math.abs(2 * Math.PI / arc2.curvature));
                let t = fs[i](s);
                t = mod(t, Math.abs(2 * Math.PI / arc1.curvature));

                console.debug("[MatrixMath] found result, t: " + t + " | s: " + s);
                if (checkResult(t,s)) {
                    console.debug("[MatrixMath] true result, t: " + t + " | s: " + s);
                    let diff = result.filter(item => {
                        //去重，如果两个点距离小于0.1，则不当做结果输出。
                        return (Math.abs(item.t - t) < 0.1) && (Math.abs(item.s - s) < 0.1)
                    })
                    if (diff.length === 0) {
                        result.push({
                            t: t,
                            s: s
                        });
                    }
                }
            }
        }
        console.debug("[MatrixMath] found result: ", result);
        return result;
    }
}

function StraightSideLineCrossArcSideLine(line1, arc2) {
    let x0 = line1.x;
    let y0 = line1.y;
    let x1 = arc2.x;
    let y1 = arc2.y;
    let cur2 = arc2.curvature;
    let alpha = line1.hdg;
    let beta = arc2.hdg;
    let poly0 = line1.poly;
    let poly1 = arc2.poly;

    function f(s) {
        return ((x1 - (1 / cur2) * Math.sin(beta) + (poly1.value(s) - 1 / cur2) * Math.sin(-1 * beta - s * cur2)) * Math.cos(alpha)
            + (y1 + (1 / cur2) * Math.cos(beta) + (poly1.value(s) - 1 / cur2) * Math.cos(-1 * beta - s * cur2)) * Math.sin(alpha)
            - x0 * Math.cos(alpha) - y0 * Math.sin(alpha))
            / (Math.pow(Math.cos(alpha), 2) + Math.pow(Math.sin(alpha), 2));
    }

    function co(s) {
        return x1 - (1 / cur2) * Math.sin(beta) + (poly1.value(s) - 1 / cur2) * Math.sin(-1 * beta - s * cur2)
            - x0 - f(s) * Math.cos(alpha) + poly0.value(f(s)) * Math.sin(alpha);
    }

    this.cross = function () {
        //TODO 两个交点求法，最多只能求得两个交点， 从0开始和从终点开始
        let s = algos.secantSolver(co, 0);
        let t = f(s);
        if (s && t >= 0 && t <= line1.length && s >= 0 && s <= arc2.length) {
            return {
                t: t,
                s: s
            };
        }
    }
}

function ArcSideLineCrossStraightSideline(arc2, line1) {
    let x0 = line1.x;
    let y0 = line1.y;
    let x1 = arc2.x;
    let y1 = arc2.y;
    let cur2 = arc2.curvature;
    let alpha = line1.hdg;
    let beta = arc2.hdg;
    let poly0 = line1.poly;
    let poly1 = arc2.poly;

    function f(s) {
        return ((x1 - (1 / cur2) * Math.sin(beta) + (poly1.value(s) - 1 / cur2) * Math.sin(-1 * beta - s * cur2)) * Math.cos(alpha)
            + (y1 + (1 / cur2) * Math.cos(beta) + (poly1.value(s) - 1 / cur2) * Math.cos(-1 * beta - s * cur2)) * Math.sin(alpha)
            - x0 * Math.cos(alpha) - y0 * Math.sin(alpha))
            / (Math.pow(Math.cos(alpha), 2) + Math.pow(Math.sin(alpha), 2));
    }

    function co(s) {
        return x1 - (1 / cur2) * Math.sin(beta) + (poly1.value(s) - 1 / cur2) * Math.sin(-1 * beta - s * cur2)
            - x0 - f(s) * Math.cos(alpha) + poly0.value(f(s)) * Math.sin(alpha);
    }

    this.cross = function () {
        let s = algos.secantSolver(co, 0);
        let t = f(s);
        if (s && t >= 0 && t <= line1.length && s >= 0 && s <= arc2.length) {
            return {
                t: s,
                s: t
            };
        }
    }
}

function StraightLineCrossStraightLine(line1, line2) {
}
function ArcLineCrossArcLine(line1, line2) {
}
function StraightLineCrossArcLine(line1, line2) {
}
function ArcLineCrossStraightLine(line1, line2) {
}
function MathCross(math1, math2) {
    if (math1 instanceof StraightSideLine) {
        if (math2 instanceof StraightSideLine) {
            return new StraightSideLineCrossStraightSideLine(math1, math2);
        }
        return new StraightSideLineCrossArcSideLine(math1, math2);
    } else {
        if (math2 instanceof StraightSideLine) {
            return new ArcSideLineCrossStraightSideline(math1, math2);
        }
        return new ArcSideLineCrossArcSideLine(math1, math2);
    }
}

export {StraightSideLine, ArcSideLine, MathCross, StraightSideLineCrossStraightSideLine, ArcSideLineCrossArcSideLine, ArcSideLineCrossStraightSideline}
