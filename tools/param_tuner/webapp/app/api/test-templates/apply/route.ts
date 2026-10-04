import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { applyArrayValuesToSystemYaml, applyTestTemplateToSystemYaml, getTestTemplate } from "@/lib/test-templates";

export const runtime = "nodejs";

export async function POST(request: NextRequest) {
  const body = await request.json();
  const id = body?.id as string | undefined;
  if (!id) {
    return NextResponse.json({ error: "id is required" }, { status: 400 });
  }
  try {
    const machine = machineOf(request);
    const template = getTestTemplate(machine, id);
    applyTestTemplateToSystemYaml(machine, template.values);
    if (template.arrayValues) applyArrayValuesToSystemYaml(machine, template.arrayValues);
    return NextResponse.json({ ok: true, name: template.name });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
