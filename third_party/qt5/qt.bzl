load("@rules_cc//cc:defs.bzl", "cc_library")

def _file_name(filePathName):
    if "/" in filePathName:
        return filePathName.rsplit("/", -1)[1]
    else:
        return filePathName

def _base_name(fileName):
    return fileName.split(".")[0]

def qt_cc_library(name, srcs, hdrs, copts = [], uis = [], res = [], res_srcs = [], normal_hdrs = [], deps = None, **kwargs):
    dep_labels = deps if deps != None else []
    resource_inputs = res_srcs if res_srcs != None else []
    for hItem in hdrs:
        base_name = _base_name(_file_name(hItem))
        cmd = """
        if grep -q Q_OBJECT $(location %s); then \
            if [ -x /usr/local/qt5/bin/moc ]; then QT_MOC=/usr/local/qt5/bin/moc; else QT_MOC=moc; fi; \
            $$QT_MOC $(location %s) -o $@ -f'%s'; \
        else \
            echo '' > $@ ; \
        fi""" % (hItem, hItem, "%s/%s" % (native.package_name(), hItem))
        native.genrule(
            name = "%s_moc" % base_name,
            srcs = [hItem],
            outs = ["moc_%s.cpp" % base_name],
            cmd = cmd,
        )
        srcs.append("moc_%s.cpp" % base_name)

    for uitem in uis:
        base_name = _base_name(_file_name(uitem))
        native.genrule(
            name = "%s_ui" % base_name,
            srcs = [uitem],
            outs = ["ui_%s.h" % base_name],
            cmd = "if [ -x /usr/local/qt5/bin/uic ]; then QT_UIC=/usr/local/qt5/bin/uic; else QT_UIC=uic; fi; $$QT_UIC $(locations %s) -o $@" % uitem,
        )
        hdrs.append("ui_%s.h" % base_name)

    for ritem in res:
        base_name = _base_name(_file_name(ritem))
        native.genrule(
            name = "%s_res" % base_name,
            srcs = [ritem] + resource_inputs,
            outs = ["res_%s.cpp" % base_name],
            cmd = "if [ -x /usr/local/qt5/bin/rcc ]; then QT_RCC=/usr/local/qt5/bin/rcc; else QT_RCC=rcc; fi; $$QT_RCC --name res --output $(OUTS) $(location %s)" % ritem,
        )
        srcs.append("res_%s.cpp" % base_name)

    hdrs = hdrs + normal_hdrs
    cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = dep_labels,
        copts = copts + ["-fPIC"],
        alwayslink = 1,
        **kwargs
    )

