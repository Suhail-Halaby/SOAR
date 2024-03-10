#Author-Suhail Halaby
#Description-ICJP Compressor CAD Program (blade profile)

import adsk.core, adsk.fusion, adsk.cam, traceback
import math
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct) 


        ##Taking Parameters
        

        ## MAKE SURE THE FUSION FILE USES THE CORRECT VARIABLE NAMES ##

        #blade profiles
        beta = (design.userParameters.itemByName('coef1'))
        betav = math.pi*beta.value/180
        rd = design.userParameters.itemByName('coef2')
        sub = design.userParameters.itemByName('nstep')
        rspool =design.userParameters.itemByName('rspool')
        #blade size
        finz = design.userParameters.itemByName('hspool')
        #base info
        boffset = design.userParameters.itemByName('h')
        thck = design.userParameters.itemByName('basethickness')
        rbase = design.userParameters.itemByName('baseradius')
        #top and length
        rtop = design.userParameters.itemByName('topradius')
        Ll = design.userParameters.itemByName('L')
        #blade thickness
        bthck = design.userParameters.itemByName('bladethickness')
        #taper offset 
        vtoff = design.userParameters.itemByName('verticaltaperoffset')
        htoff = design.userParameters.itemByName('horizontaltaperoffset')
        tat = design.userParameters.itemByName('taperattop')


        # governing blade function
        ang = lambda zval: math.tan(betav)*(Ll.value**2-((finz.value-zval)-Ll.value)**2)/(2*Ll.value*rspool.value)
        
        
        Z = [finz.value]
        theta = [ang(finz.value)]
        Y = [rspool.value*math.sin(ang(finz.value))]
        X = [rspool.value*math.cos(ang(finz.value))]
        ZZ = [-boffset.value]
        RR = [-rbase.value]
        RR2 = [-rbase.value-htoff.value]
        kval = Ll.value
        aval = rbase.value-rtop.value
        rval = rtop.value
        rval2 = rtop.value+tat.value
        # old function (finz.value/rd.value)*math.tan(math.radians(beta.value))/2*(zval-finz.value)**2
       
       
        # Coordinate Generation For-Loop
        n = int(sub.value)
        for i in range (1,n+1):
            ##Blade
            Z.append(Z[i-1]-(finz.value/n))
            # actual profile function
            theta.append(ang(Z[i]))
            # X-Y conversion
            Y.append(rspool.value*math.sin(theta[i]))
            X.append(rspool.value*math.cos(theta[i]))

            ##Inner Profile
            ZZ.append(ZZ[i-1]+((kval)/n))
            RR.append(math.sqrt(aval**2-(aval**2*((ZZ[i]+boffset.value)-kval)**2)/kval**2)-aval-rval)
            RR2.append(math.sqrt(aval**2-(aval**2*((ZZ[i]+boffset.value)-kval)**2)/kval**2)-aval-rval2-htoff.value)



        # Point Display
        ui.messageBox('theta increments\n {}'.format(theta))
        # Point and Line Plotter

        rootComp = design.rootComponent
        # Create a new sketch on the xy plane.
        sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch2 = rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch3 = rootComp.sketches.add(rootComp.xYConstructionPlane)
        # Create an object collection for the points.
        points = adsk.core.ObjectCollection.create()
        points2 = adsk.core.ObjectCollection.create()
        points3 = adsk.core.ObjectCollection.create()

        for i in range (0,n+1):
            points.add(adsk.core.Point3D.create(X[i], Y[i], Z[i]))
            points2.add(adsk.core.Point3D.create(RR[i],0,ZZ[i]))
            points3.add(adsk.core.Point3D.create(RR2[i], 0 , ZZ[i]+vtoff.value))

        ## Splines
        spline = sketch.sketchCurves.sketchFittedSplines.add(points)
        spline2 = sketch2.sketchCurves.sketchFittedSplines.add(points2)
        spline3 = sketch3.sketchCurves.sketchFittedSplines.add(points3)
        # Get spline fit points
        fitPoints = spline.fitPoints
        fitPoints2 = spline2.fitPoints
        fitPoints3 = spline3.fitPoints
        # Get the second fit point
        fitPoint = fitPoints.item(1)
        fitPoint2 = fitPoints2.item(1)
        fitPoint3 = fitPoints3.item(1)
        # If there is no the relative tangent handle, activate the tangent handle
        line = spline.getTangentHandle(fitPoint)
        line2 = spline2.getTangentHandle(fitPoint2)
        line3 = spline3.getTangentHandle(fitPoint3)
        if line is None:
             line = spline.activateTangentHandle(fitPoint)
        if line2 is None:
             line2 = spline2.activateTangentHandle(fitPoint2)
        if line3 is None:
             line3 = spline3.activateTangentHandle(fitPoint3)          
        # Get the tangent handle           
        gottenLine = spline.getTangentHandle(fitPoint)
        gottenLine2 = spline2.getTangentHandle(fitPoint2)
        gottenLine3 = spline3.getTangentHandle(fitPoint3)
        # Delete the tangent handle
        gottenLine.deleteMe()
        gottenLine2.deleteMe()
        gottenLine3.deleteMe()
        # Activate the curvature handle
        # If the curvature handle activated. the relative tangentHandle is activated automatically
        activatedArc= spline.activateCurvatureHandle(fitPoint)
        activatedArc2= spline2.activateCurvatureHandle(fitPoint2)
        activatedArc3= spline3.activateCurvatureHandle(fitPoint3)
        # Get curvature handle and tangent handle
        gottenArc= spline.getCurvatureHandle(fitPoint)
        gottenLine = spline.getTangentHandle(fitPoint)
        gottenArc2= spline2.getCurvatureHandle(fitPoint2)
        gottenLine2 = spline2.getTangentHandle(fitPoint2)
        gottenArc3= spline3.getCurvatureHandle(fitPoint3)
        gottenLine3 = spline3.getTangentHandle(fitPoint3)
        # Delete curvature handle
        gottenArc.deleteMe();
        gottenArc2.deleteMe();
        gottenArc3.deleteMe();
    




        ## Compressor Geometry
        sketches = rootComp.sketches
        xyPlane = rootComp.xYConstructionPlane
        sketch = sketches.add(xyPlane)

        # Draw some circles (pretty useless unfortunately)
        circles = sketch.sketchCurves.sketchCircles
        circle1 = circles.addByCenterRadius(adsk.core.Point3D.create(0, 0, -boffset.value), rbase.value)
        circle2 = circles.addByCenterRadius(adsk.core.Point3D.create(0, 0, -boffset.value-thck.value), rbase.value)
        circle3 = circles.addByCenterRadius(adsk.core.Point3D.create(0, 0, Ll.value-boffset.value), rtop.value)
        # Center-line
        lines = sketch.sketchCurves.sketchLines
        lines2 = sketch2.sketchCurves.sketchLines
        lines3 = sketch3.sketchCurves.sketchLines
        #centlines = lines.addTwoPointRectangle(adsk.core.Point3D.create(0, 0, 0), adsk.core.Point3D.create(0, 0, finz.value))    
            
        # Radial Lines
        #bladeprofm = lines.addByTwoPoints(adsk.core.Point3D.create(X[n], Y[n], 0), adsk.core.Point3D.create(0, 0, 0))
        #bladeprofb = lines.addByTwoPoints(adsk.core.Point3D.create(X[n], Y[n], -boffset.value), adsk.core.Point3D.create(0, 0, -boffset.value))
        #bladeproft = lines.addByTwoPoints(adsk.core.Point3D.create(X[0], Y[0], finz.value), adsk.core.Point3D.create(0, 0, finz.value))
        
        # Blade profiles
        bladet = lines.addThreePointRectangle(adsk.core.Point3D.create(rspool.value, -bthck.value/2, finz.value), adsk.core.Point3D.create(0, -bthck.value/2,finz.value),adsk.core.Point3D.create(0, bthck.value/2,finz.value))
        bladem1 = lines.addByTwoPoints(adsk.core.Point3D.create(X[n]-(bthck.value/2)*math.sin(theta[n]),Y[n]+(bthck.value/2)*math.cos(theta[n]), 0 ), adsk.core.Point3D.create(-(bthck.value/2)*math.sin(theta[n]),(bthck.value/2)*math.cos(theta[n]),0))
        bladem2 = lines.addByTwoPoints(adsk.core.Point3D.create(X[n]+(bthck.value/2)*math.sin(theta[n]),Y[n]-(bthck.value/2)*math.cos(theta[n]), 0), adsk.core.Point3D.create((bthck.value/2)*math.sin(theta[n]),-(bthck.value/2)*math.cos(theta[n]),0))
        bladem3 = lines.addByTwoPoints(adsk.core.Point3D.create(X[n]-(bthck.value/2)*math.sin(theta[n]),Y[n]+(bthck.value/2)*math.cos(theta[n]), 0),adsk.core.Point3D.create(X[n]+(bthck.value/2)*math.sin(theta[n]),Y[n]-(bthck.value/2)*math.cos(theta[n]),0))
        bladem4 = lines.addByTwoPoints(adsk.core.Point3D.create(-(bthck.value/2)*math.sin(theta[n]),(bthck.value/2)*math.cos(theta[n]), 0),adsk.core.Point3D.create((bthck.value/2)*math.sin(theta[n]),-(bthck.value/2)*math.cos(theta[n]),0))
        
        # Inner commpressor profile (to be rotated)
        prof1 = lines2.addByTwoPoints(adsk.core.Point3D.create(0,0,-boffset.value-thck.value), adsk.core.Point3D.create(-rbase.value,0,-boffset.value-thck.value))
        prof2 = lines2.addByTwoPoints(adsk.core.Point3D.create(-rbase.value,0,-boffset.value), adsk.core.Point3D.create(-rbase.value,0,-boffset.value-thck.value))
        prof3 = lines2.addByTwoPoints(adsk.core.Point3D.create(0,0,Ll.value-boffset.value), adsk.core.Point3D.create(-rtop.value,0,Ll.value-boffset.value))
        prof4 = lines2.addByTwoPoints(adsk.core.Point3D.create(0,0,-(thck.value+boffset.value)),adsk.core.Point3D.create(0,0,Ll.value-boffset.value))
        
        # Taper Peofile
        t1 = lines3.addByTwoPoints(adsk.core.Point3D.create(RR2[0],0,ZZ[0]),adsk.core.Point3D.create(RR2[0]-1,0,ZZ[0]))
        t2 = lines3.addByTwoPoints(adsk.core.Point3D.create(RR2[n],0,ZZ[n]),adsk.core.Point3D.create(RR2[0]-1,0,ZZ[n]))
        t3 = lines3.addByTwoPoints(adsk.core.Point3D.create(RR2[0]-1,0,ZZ[0]),adsk.core.Point3D.create(RR2[0]-1,0,ZZ[n]))
        
        
        
        # Rotation about axis
        # Double extrude
        # Rotational pattern
    
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
