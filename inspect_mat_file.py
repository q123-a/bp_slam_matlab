"""
检查 scenarioCleanM2_new.mat 文件的结构
Inspect the structure of scenarioCleanM2_new.mat
"""
import scipy.io
import numpy as np

# 加载 MAT 文件
print("加载 scenarioCleanM2_new.mat...")
mat_data = scipy.io.loadmat('scenarioCleanM2_new.mat')

print("\n=== MAT 文件中的变量 ===")
for key in mat_data.keys():
    if not key.startswith('__'):
        print(f"\n变量名: {key}")
        print(f"  类型: {type(mat_data[key])}")
        print(f"  形状: {mat_data[key].shape if hasattr(mat_data[key], 'shape') else 'N/A'}")
        print(f"  数据类型: {mat_data[key].dtype if hasattr(mat_data[key], 'dtype') else 'N/A'}")

# 详细检查 dataVA
if 'dataVA' in mat_data:
    print("\n=== dataVA 详细结构 ===")
    dataVA = mat_data['dataVA']
    print(f"dataVA 类型: {type(dataVA)}")
    print(f"dataVA 形状: {dataVA.shape}")
    print(f"dataVA 数据类型: {dataVA.dtype}")

    # 如果是对象数组，检查每个元素
    if dataVA.dtype == np.object_:
        print(f"\ndataVA 是对象数组，包含 {dataVA.size} 个元素")
        for i in range(min(dataVA.size, 5)):  # 只显示前5个
            print(f"\n  元素 [{i}]:")
            elem = dataVA.flat[i]
            print(f"    类型: {type(elem)}")
            if hasattr(elem, 'shape'):
                print(f"    形状: {elem.shape}")
            if hasattr(elem, 'dtype'):
                print(f"    数据类型: {elem.dtype}")

            # 如果是结构体，显示字段
            if isinstance(elem, np.ndarray) and elem.dtype.names:
                print(f"    字段: {elem.dtype.names}")
                for field in elem.dtype.names:
                    field_data = elem[field][0, 0]
                    print(f"      {field}: 形状={field_data.shape if hasattr(field_data, 'shape') else 'N/A'}")

# 详细检查 trueTrajectory
if 'trueTrajectory' in mat_data:
    print("\n=== trueTrajectory 详细结构 ===")
    trueTrajectory = mat_data['trueTrajectory']
    print(f"trueTrajectory 类型: {type(trueTrajectory)}")
    print(f"trueTrajectory 形状: {trueTrajectory.shape}")
    print(f"trueTrajectory 数据类型: {trueTrajectory.dtype}")
    print(f"前5列:\n{trueTrajectory[:, :5]}")

print("\n检查完成！")
