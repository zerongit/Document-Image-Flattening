from abaqus import *
from abaqusConstants import *
session.Viewport(name='Viewport: 1', origin=(0.0, 0.0), width=310.710388183594, 
    height=168.007415771484)
session.viewports['Viewport: 1'].makeCurrent()
session.viewports['Viewport: 1'].maximize()
from caeModules import *
from driverUtils import executeOnCaeStartup
executeOnCaeStartup()
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=ON)
#%%
def str2array(s):
    s=s[1:-2]
    # print(s)
    l=s.split()
    res=[eval(i) for i in l]
    return res

f=open('./config.txt','r')
lines=f.readlines()
f.close()
# conf=[]
# for l in lines:
#     r=l.split(',')[-1]
#     conf.append(r)

path=lines[2].split(',')[1][:-1]
# path=conf[3][:-1]
path=path.replace('\\','/')
path=path.strip('\r')

#%%
# path = r"E:\\weekly report\\osFEM unfold\\inte\\temp"
input_inp ='/test0321.inp'
obd_name ='/test_v2.odb'
cae_name = '/test_v2'
result_file ='/test_v2.rpt'

params='/params.txt'

file=open(path+params,'r')
lines=file.readlines()
file.close()
param=[]
for l in lines:
    r=l.split(':')
    param.append(r[1])

# node_num=12800
# element_num=12573

node_num=int(param[0])
element_num=int(param[1])
A=str2array(param[2])
mean=str2array(param[3])
theta=float(param[4])

cellsize=float(param[6])

#%%
# set working directory
import os
os.chdir(path)


def xyz_calc(file):
    with open(file, 'r') as inpfile:
        lines = inpfile.readlines()
    node = lines.index('*Node\n')
    element = lines.index('*Element, type=S3R\n')
    X = []
    Y = []
    Z = []
    for i in range(node+1, element):
        line = lines[i].split(',')[1: ]
        X.append(float(line[0]))
        Y.append(float(line[1]))
        Z.append(float(line[2]))
    return X, Y, Z


# import point cloud data
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.ModelFromInputFile(name='test', 
    inputFileName=path+input_inp)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    optimizationTasks=OFF, geometricRestrictions=OFF, stopConditions=OFF)
a = mdb.models['test'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
del mdb.models['Model-1']
a = mdb.models['test'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)

# create the two plane components
x, y, z = xyz_calc(path+input_inp)
p1 = mdb.models['test'].parts['PART-1']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
s = mdb.models['test'].ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
s.rectangle(point1=(min(x)*3, min(y)*3), point2=(max(x)*3, max(y)*3))
p = mdb.models['test'].Part(name='Part-2', dimensionality=THREE_D, 
    type=DEFORMABLE_BODY)
p = mdb.models['test'].parts['Part-2']
p.BaseShell(sketch=s)
s.unsetPrimaryObject()
p = mdb.models['test'].parts['Part-2']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
del mdb.models['test'].sketches['__profile__']

s1 = mdb.models['test'].ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
s1.setPrimaryObject(option=STANDALONE)
s1.rectangle(point1=(min(x)*3, min(y)*3), point2=(max(x)*3, max(y)*3))
p = mdb.models['test'].Part(name='Part-3', dimensionality=THREE_D, 
    type=DEFORMABLE_BODY)
p = mdb.models['test'].parts['Part-3']
p.BaseShell(sketch=s1)
s1.unsetPrimaryObject()
p = mdb.models['test'].parts['Part-3']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
del mdb.models['test'].sketches['__profile__']

# define the two meterial properties
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=OFF)
mdb.models['test'].Material(name='paper')
mdb.models['test'].materials['paper'].Density(table=((1180.0, ), ))
# mdb.models['test'].materials['paper'].Density(table=((0.0, ), ))
mdb.models['test'].materials['paper'].Elastic(table=((1.0, 1.0), ))
mdb.models['test'].materials['paper'].Plastic(table=((16500000.0, 
    0.0), (47000000.0, 0.062)))
mdb.models['test'].materials['paper'].plastic.Potential(table=((1.0, 0.355, 
    0.355, 0.8, 0.207846, 0.207846), ))
mdb.models['test'].materials['paper'].elastic.setValues(
    type=ENGINEERING_CONSTANTS, table=((6500000000.0*10, 3000000000.0*10, 
    200000000.0*10, 0.3*1, 0.0*1, 0.0*1, 1000000000.0*1, 60000000.0*1, 60000000.0*1), ))
# mdb.models['test'].materials['paper'].elastic.setValues(
#     type=ENGINEERING_CONSTANTS, table=((0.01, 0.01, 
#     0.01, 0.01, 0.0, 0.0, 1000000000.0, 60000000.0, 60000000.0), ))

mdb.models['test'].Material(name='steel')
mdb.models['test'].materials['steel'].Density(table=((7850.0, ), ))
mdb.models['test'].materials['steel'].Elastic(table=((200000000000.0, 0.3), 
    ))

# define and assign the sections
mdb.models['test'].HomogeneousShellSection(name='Section-1', 
    preIntegrate=OFF, material='steel', thicknessType=UNIFORM, thickness=0.01, 
    thicknessField='', idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
    thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
    integrationRule=SIMPSON, numIntPts=5)
mdb.models['test'].HomogeneousShellSection(name='Section-2', 
    preIntegrate=OFF, material='paper', thicknessType=UNIFORM, thickness=0.01, 
    thicknessField='', idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
    thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
    integrationRule=SIMPSON, numIntPts=5)

p = mdb.models['test'].parts['PART-1']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
p = mdb.models['test'].parts['PART-1']
e = p.elements

# elements = e.getSequenceFromMask(mask=('[#ffffffff:417 ]', ), )
elements=e[:element_num]

region = p.Set(elements=elements, name='Set-1')
p = mdb.models['test'].parts['PART-1']
p.SectionAssignment(region=region, sectionName='Section-2', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)

p = mdb.models['test'].parts['Part-2']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
p = mdb.models['test'].parts['Part-2']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1 ]', ), )
region = p.Set(faces=faces, name='Set-1')
p = mdb.models['test'].parts['Part-2']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)

p = mdb.models['test'].parts['Part-3']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
p = mdb.models['test'].parts['Part-3']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1 ]', ), )
region = p.Set(faces=faces, name='Set-1')
p = mdb.models['test'].parts['Part-3']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)

# assemble
a = mdb.models['test'].rootAssembly
a.regenerate()
a = mdb.models['test'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)

a = mdb.models['test'].rootAssembly
p = mdb.models['test'].parts['Part-2']
a.Instance(name='Part-2-1', part=p, dependent=ON)
p = mdb.models['test'].parts['Part-3']
a.Instance(name='Part-3-1', part=p, dependent=ON)

a = mdb.models['test'].rootAssembly
a.translate(instanceList=('Part-3-1', ), vector=(0.0, 0.0, max(z)*3))
if min(z) <=0:
    a = mdb.models['test'].rootAssembly
    a.translate(instanceList=('Part-2-1', ), vector=(0.0, 0.0, min(z)*3))
    d = (max(z)-min(z))*(3-0.1)
else:
    d = max(z)*(3-0.1)

# create analysis step
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
mdb.models['test'].ExplicitDynamicsStep(name='Step-1', previous='Initial')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-1')

# define interactions
session.viewports['Viewport: 1'].assemblyDisplay.setValues(interactions=ON, 
    constraints=ON, connectors=ON, engineeringFeatures=ON, 
    adaptiveMeshConstraints=OFF)

mdb.models['test'].ContactProperty('IntProp-1')
mdb.models['test'].interactionProperties['IntProp-1'].TangentialBehavior(
    formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
    pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((
    0.001, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
    fraction=0.005, elasticSlipStiffness=None)
mdb.models['test'].interactionProperties['IntProp-1'].NormalBehavior(
    pressureOverclosure=EXPONENTIAL, table=((750.0, 0.0), (0.0, 0.0001)), 
    maxStiffness=None, constraintEnforcementMethod=DEFAULT)

a = mdb.models['test'].rootAssembly
s1 = a.instances['Part-3-1'].faces
side2Faces1 = s1.getSequenceFromMask(mask=('[#1 ]', ), )
region1=a.Surface(side2Faces=side2Faces1, name='m_Surf-1')
a = mdb.models['test'].rootAssembly
s1 = a.instances['PART-1-1'].elements

# side2Elements1 = s1.getSequenceFromMask(mask=('[#ffffffff:417 ]', ), )
side2Elements1=s1[:element_num]

region2=a.Surface(side2Elements=side2Elements1, name='s_Surf-1')
mdb.models['test'].SurfaceToSurfaceContactExp(name ='Int-1', 
    createStepName='Step-1', master = region1, slave = region2, 
    mechanicalConstraint=KINEMATIC, sliding=FINITE, 
    interactionProperty='IntProp-1', initialClearance=OMIT, datumAxis=None, 
    clearanceRegion=None)

a = mdb.models['test'].rootAssembly
s1 = a.instances['Part-2-1'].faces
side1Faces1 = s1.getSequenceFromMask(mask=('[#1 ]', ), )
region1=a.Surface(side1Faces=side1Faces1, name='m_Surf-3')
a = mdb.models['test'].rootAssembly
s1 = a.instances['PART-1-1'].elements

# side1Elements1 = s1.getSequenceFromMask(mask=('[#ffffffff:417 ]', ), )
side1Elements1=s1[:element_num]

region2=a.Surface(side1Elements=side1Elements1, name='s_Surf-3')
mdb.models['test'].SurfaceToSurfaceContactExp(name ='Int-2', 
    createStepName='Step-1', master = region1, slave = region2, 
    mechanicalConstraint=KINEMATIC, sliding=FINITE, 
    interactionProperty='IntProp-1', initialClearance=OMIT, datumAxis=None, 
    clearanceRegion=None)

# define rigid constaints
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=ON)

p1 = mdb.models['test'].parts['Part-2']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p = mdb.models['test'].parts['Part-2']
p.ReferencePoint(point=(0.0, 0.0, 0.0))
p1 = mdb.models['test'].parts['Part-3']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p = mdb.models['test'].parts['Part-3']
p.ReferencePoint(point=(0.0, 0.0, 0.0))

a1 = mdb.models['test'].rootAssembly
a1.regenerate()
a = mdb.models['test'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)

a = mdb.models['test'].rootAssembly
f1 = a.instances['Part-3-1'].faces
faces1 = f1.getSequenceFromMask(mask=('[#1 ]', ), )
region2=a.Set(faces=faces1, name='b_Set-1')
a = mdb.models['test'].rootAssembly
r1 = a.instances['Part-3-1'].referencePoints
refPoints1=(r1[3], )
region1=regionToolset.Region(referencePoints=refPoints1)
mdb.models['test'].RigidBody(name='Constraint-1', refPointRegion=region1, 
    bodyRegion=region2)

a = mdb.models['test'].rootAssembly
f1 = a.instances['Part-2-1'].faces
faces1 = f1.getSequenceFromMask(mask=('[#1 ]', ), )
region2=a.Set(faces=faces1, name='b_Set-3')
a = mdb.models['test'].rootAssembly
r1 = a.instances['Part-2-1'].referencePoints
refPoints1=(r1[3], )
region1=regionToolset.Region(referencePoints=refPoints1)
mdb.models['test'].RigidBody(name='Constraint-2', refPointRegion=region1, 
    bodyRegion=region2)

# define loads
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, interactions=OFF, constraints=OFF, 
    engineeringFeatures=OFF)
mdb.models['test'].TabularAmplitude(name='Amp-1', timeSpan=STEP, 
    smooth=SOLVER_DEFAULT, data=((0.0, 0.0), (1.0, 1.0)))

a = mdb.models['test'].rootAssembly
r1 = a.instances['Part-3-1'].referencePoints
refPoints1=(r1[3], )
region = a.Set(referencePoints=refPoints1, name='Set-5')
mdb.models['test'].EncastreBC(name='BC-1', createStepName='Initial', 
    region=region, localCsys=None)

a = mdb.models['test'].rootAssembly
r1 = a.instances['Part-2-1'].referencePoints
refPoints1=(r1[3], )
region = a.Set(referencePoints=refPoints1, name='Set-6')
mdb.models['test'].DisplacementBC(name='BC-2', createStepName='Initial', 
    region=region, u1=0.0, u2=0.0, u3=0.0, ur1=0.0, ur2=0.0, ur3=0.0, 
    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
    localCsys=None)
mdb.models['test'].boundaryConditions['BC-2'].setValuesInStep(
    stepName='Step-1', u3=d, amplitude='Amp-1')

session.viewports['Viewport: 1'].view.setValues(nearPlane=14.838, 
    farPlane=24.8725, width=0.202078, height=0.115169, viewOffsetX=0.693798, 
    viewOffsetY=1.99165)
a = mdb.models['test'].rootAssembly
n1 = a.instances['PART-1-1'].nodes

# nodes1 = n1.getSequenceFromMask(mask=('[#0:215 #80 ]', ), )
nodes1=n1[:1]

region = a.Set(nodes=nodes1, name='Set-7')
mdb.models['test'].DisplacementBC(name='BC-3', createStepName='Initial', 
    region=region, u1=0.0, u2=0.0, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
    localCsys=None)

# divide mesh
session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=ON, 
    interactions=OFF, constraints=OFF, connectors=OFF, engineeringFeatures=OFF)
session.viewports['Viewport: 1'].assemblyDisplay.meshOptions.setValues(
    meshTechnique=ON)

p = mdb.models['test'].parts['Part-3']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
session.viewports['Viewport: 1'].partDisplay.setValues(mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=OFF)
p = mdb.models['test'].parts['Part-3']
p.seedPart(size=2*cellsize, deviationFactor=0.1, minSizeFactor=0.1)
p = mdb.models['test'].parts['Part-3']
p.generateMesh()

p = mdb.models['test'].parts['Part-2']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
p = mdb.models['test'].parts['Part-2']
p.seedPart(size=2*cellsize, deviationFactor=0.1, minSizeFactor=0.1)
p = mdb.models['test'].parts['Part-2']
p.generateMesh()

# create work and save inp
a = mdb.models['test'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=OFF)
session.viewports['Viewport: 1'].assemblyDisplay.meshOptions.setValues(
    meshTechnique=OFF)
mdb.Job(name='test_v2', model='test', description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, explicitPrecision=SINGLE, 
    nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF, 
    contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='', 
    resultsFormat=ODB, parallelizationMethodExplicit=DOMAIN, numDomains=1, 
    activateLoadBalancing=False, multiprocessingMode=DEFAULT, numCpus=1)
mdb.jobs['test_v2'].writeInput(consistencyChecking=OFF)

# submit work and wait to complete
mdb.jobs['test_v2'].submit(consistencyChecking=OFF)
mdb.jobs['test_v2'].waitForCompletion()

# save calculation results
session.viewports['Viewport: 1'].setValues(displayedObject=None)
o1 = session.openOdb(name=path+obd_name)
session.viewports['Viewport: 1'].setValues(displayedObject=o1)

leaf = dgo.LeafFromPartInstance(partInstanceName=('PART-1-1', ))
session.viewports['Viewport: 1'].odbDisplay.displayGroup.replace(leaf=leaf)
dg = session.viewports['Viewport: 1'].odbDisplay.displayGroup
dg = session.DisplayGroup(name='DisplayGroup-2', objectToCopy=dg)
session.viewports['Viewport: 1'].odbDisplay.setValues(visibleDisplayGroups=(dg, 
    ))
session.viewports['Viewport: 1'].odbDisplay.displayGroupInstances['DisplayGroup-2'].setValues(
    lockOptions=OFF)

session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].odbDisplay.setPrimaryVariable(
    variableLabel='U', outputPosition=NODAL, refinement=(INVARIANT, 
    'Magnitude'), )

odb = session.odbs[path+obd_name]
session.fieldReportOptions.setValues(printTotal=OFF, printMinMax=OFF)
session.writeFieldReport(fileName=path+result_file, append=OFF, 
    sortItem='Node Label', odb=odb, step=0, frame=20, outputPosition=NODAL, 
    variable=(('U', NODAL, ((COMPONENT, 'U1'), (COMPONENT, 'U2'), (COMPONENT, 
    'U3'), )), ))

mdb.saveAs(pathName=path+cae_name)