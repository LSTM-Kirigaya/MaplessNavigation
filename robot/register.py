from logging import warning
import importlib

MODEL_MODULES = ["models"]
ALL_MODULES = [("model", MODEL_MODULES)]

class Register(object):
    def __init__(self, register_name):
        self._dict = {}
        self._name = register_name
    
    def register(self, target):
        """
            We will use the function as a decorator to register a concrete BaseScene class

        """
        def add_register_item(key, value):
            if not callable(value):
                raise Exception(f"register object must be callable! But receice:{value} is not callable!")
            if key is None:
                key = value.__name__
            if key in self._dict:
                warning(f"{value.__name__} has been registered before, so we will overriden it")
            self._dict[key] = value
            return value

        if callable(target):            # 如果传入的target可调用，说明是函数或者类，直接注册
            return add_register_item(None, target)
        else:                           # 如果不可调用，说明额外说明了注册的可调用对象的名字
            return lambda x : add_register_item(target, x)
    
    def __getitem__(self, key):
        return self._dict[key]
    
    def __contains__(self, key):
        return key in self._dict
    
    def keys(self):
        return self._dict.keys()
    def values(self):
        return self._dict.values()
    def items(self):
        return self._dict.items()

class Registers(object):
    def __init__(self):
        raise RuntimeError("Registers is not intended to be instantiated")
    
    scenes = Register("scenes")

def _handle_errors(errors : list):
    """
        process the error happened during import
    """
    if not errors:
        return
    for name, err in errors:
        warning(f"Module {name} import failed: {err}")

def import_all_modules_for_register(custom_module_paths : list = None):
    # 将全局导入模块加入导入列表中
    modules = [base_dir + "." + name for base_dir, modules in ALL_MODULES for name in modules]
    modules += custom_module_paths if isinstance(custom_module_paths, list) else []
    # 导入
    errors = []
    for module in modules:
        try:
            importlib.import_module(module)
        except ImportError as error:
            errors.append((module, error))
    _handle_errors(errors)
